// ============================================================================
// Módulo: montgomery_multiplier.v
// Algoritmo: CIOS Montgomery Multiplication (Word-Based, W = 16 bits)
// Optimizado para: Intel Cyclone IV (Inferencia nativa de DSP 18x18)
// Operación: Result = (A * B * R^-1) mod P
// ============================================================================

module montgomery_multiplier #(
    parameter WIDTH     = 256,
    parameter WORD_SIZE = 16,
    parameter NUM_WORDS = WIDTH / WORD_SIZE // 256 / 16 = 16 palabras
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,
    
    // Entradas de datos (256 bits)
    input  wire [WIDTH-1:0]     a,
    input  wire [WIDTH-1:0]     b,
    input  wire [WIDTH-1:0]     p,
    
    // Constante n_prime0 = (-P^-1) mod 2^16
    // Para secp256k1 con W=16, n_prime0 = 16'h2ECD
    input  wire [WORD_SIZE-1:0] n_prime0,
    
    // Salida
    output reg  [WIDTH-1:0]     result
);

    // ------------------------------------------------------------------------
    // Estados de la FSM
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 3'b000;
    localparam ST_MULT_ACC   = 3'b001;
    localparam ST_M_CALC     = 3'b010;
    localparam ST_REDUCE     = 3'b011;
    localparam ST_NEXT_WORD  = 3'b100;
    localparam ST_FINAL_SUB  = 3'b101;
    localparam ST_DONE       = 3'b110;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Registros Internos y Acumuladores
    // ------------------------------------------------------------------------
    reg [WORD_SIZE-1:0] t [0:NUM_WORDS]; // Memoria temporal T (17 palabras de 16 bits)
    reg [35:0]          acc;              // Acumulador de 36 bits (evita overflow)
    reg [WORD_SIZE-1:0] m;                // Factor de reducción m = (T[0] * n_prime0) mod 2^16
    
    // Índices de bucles
    reg [4:0] i; // Índice palabra exterior (0 a 15)
    reg [4:0] j; // Índice palabra interior (0 a 15)

    // Extracción de palabras
    wire [WORD_SIZE-1:0] a_word = a[i*WORD_SIZE +: WORD_SIZE];
    wire [WORD_SIZE-1:0] b_word = b[j*WORD_SIZE +: WORD_SIZE];
    wire [WORD_SIZE-1:0] p_word = p[j*WORD_SIZE +: WORD_SIZE];

    // Señal intermedia de multiplicación para evitar truncamiento implícito (Warning 10230)
    wire [31:0] m_prod = t[0] * n_prime0;

    integer k;

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= ST_IDLE;
            done    <= 1'b0;
            busy    <= 1'b0;
            result  <= {WIDTH{1'b0}};
            i       <= 5'd0;
            j       <= 5'd0;
            acc     <= 36'd0;
            m       <= 16'd0;
            for (k = 0; k <= NUM_WORDS; k = k + 1) begin
                t[k] <= 16'd0;
            end
        end else begin
            case (state)
                
                // ------------------------------------------------------------
                // IDLE: Esperar pulso de inicio
                // ------------------------------------------------------------
                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        i     <= 5'd0;
                        j     <= 5'd0;
                        acc   <= 36'd0;
                        for (k = 0; k <= NUM_WORDS; k = k + 1) begin
                            t[k] <= 16'd0;
                        end
                        state <= ST_MULT_ACC;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                // ST_MULT_ACC: Paso 1 de CIOS - Acumular A[i] * B[j] + T[j]
                // ------------------------------------------------------------
                ST_MULT_ACC: begin
                    // acc = T[j] + A[i]*B[j] + carry
                    acc <= t[j] + (a_word * b_word) + (acc >> WORD_SIZE);
                    t[j] <= acc[WORD_SIZE-1:0];

                    if (j == NUM_WORDS - 1) begin
                        j     <= 5'd0;
                        state <= ST_M_CALC;
                    end else begin
                        j     <= j + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // ST_M_CALC: Guardar último acarreo y calcular factor m
                // ------------------------------------------------------------
                ST_M_CALC: begin
                    // Guardar palabra final de la fase de acumulación
                    acc  <= t[NUM_WORDS] + (acc >> WORD_SIZE);
                    t[NUM_WORDS] <= acc[WORD_SIZE-1:0];
                    
                    // m = (T[0] * n_prime0) mod 2^16
                    m    <= m_prod[WORD_SIZE-1:0];
                    j    <= 5'd0;
                    state<= ST_REDUCE;
                end

                // ------------------------------------------------------------
                // ST_REDUCE: Paso 2 de CIOS - Reducción m * P[j] + T[j]
                // ------------------------------------------------------------
                ST_REDUCE: begin
                    // acc = T[j] + m*P[j] + carry
                    acc <= t[j] + (m * p_word) + (acc >> WORD_SIZE);
                    
                    if (j > 0) begin
                        t[j-1] <= acc[WORD_SIZE-1:0]; // Desplazamiento de palabra hacia la derecha
                    end

                    if (j == NUM_WORDS - 1) begin
                        j     <= 5'd0;
                        state <= ST_NEXT_WORD;
                    end else begin
                        j     <= j + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // ST_NEXT_WORD: Desplazamiento final de palabra e incremento de i
                // ------------------------------------------------------------
                ST_NEXT_WORD: begin
                    acc <= t[NUM_WORDS] + (acc >> WORD_SIZE);
                    t[NUM_WORDS-1] <= acc[WORD_SIZE-1:0];
                    t[NUM_WORDS]   <= (acc >> WORD_SIZE);

                    if (i == NUM_WORDS - 1) begin
                        state <= ST_FINAL_SUB;
                    end else begin
                        i     <= i + 1'b1;
                        acc   <= 36'd0;
                        state <= ST_MULT_ACC;
                    end
                end

                // ------------------------------------------------------------
                // ST_FINAL_SUB: Sustracción condicional si T >= P
                // ------------------------------------------------------------
                ST_FINAL_SUB: begin
                    // Reconstruir vector completo de 256 bits desde t[]
                    begin: build_res
                        reg [WIDTH:0] temp_t;
                        reg [WIDTH:0] temp_sub;
                        integer x;
                        temp_t = 0;
                        for (x = 0; x < NUM_WORDS; x = x + 1) begin
                            temp_t[x*WORD_SIZE +: WORD_SIZE] = t[x];
                        end
                        temp_t[WIDTH] = t[NUM_WORDS][0];

                        temp_sub = temp_t - p;

                        // Si T >= P, Result = T - P; en otro caso, Result = T
                        if (temp_t >= p) begin
                            result <= temp_sub[WIDTH-1:0];
                        end else begin
                            result <= temp_t[WIDTH-1:0];
                        end
                    end
                    state <= ST_DONE;
                end

                // ------------------------------------------------------------
                // ST_DONE: Notificar finalización
                // ------------------------------------------------------------
                ST_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule