// ============================================================================
// Módulo: montgomery_multiplier.v (CORREGIDO)
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
    input  wire [WORD_SIZE-1:0] n_prime0,
    
    // Salida
    output reg  [WIDTH-1:0]     result
);

    // ------------------------------------------------------------------------
    // Estados de la FSM (Codificación Única)
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 3'd0;
    localparam ST_MULT_ACC   = 3'd1;
    localparam ST_M_CALC     = 3'd2;
    localparam ST_REDUCE     = 3'd3;
    localparam ST_NEXT_WORD  = 3'd4;
    localparam ST_FINAL_SUB  = 3'd5;
    localparam ST_DONE       = 3'd6;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Registros Internos y Acumuladores
    // ------------------------------------------------------------------------
    reg [WORD_SIZE-1:0] t [0:NUM_WORDS]; // Memoria temporal T (17 palabras)
    reg [35:0]          carry;           // Acumulador de acarreo de 36 bits
    reg [WORD_SIZE-1:0] m;               // Factor m
    
    // Índices de bucles
    reg [4:0] i; // Índice palabra exterior (0 a 15)
    reg [4:0] j; // Índice palabra interior (0 a 16)

    // Extracción segura de palabras de entrada con protección de rango
    wire [WORD_SIZE-1:0] a_word = a[i*WORD_SIZE +: WORD_SIZE];
    wire [WORD_SIZE-1:0] b_word = (j < NUM_WORDS) ? b[j*WORD_SIZE +: WORD_SIZE] : {WORD_SIZE{1'b0}};
    wire [WORD_SIZE-1:0] p_word = (j < NUM_WORDS) ? p[j*WORD_SIZE +: WORD_SIZE] : {WORD_SIZE{1'b0}};

    // Cálculos combinacionales intermediarios para evitar condiciones de carrera
    wire [35:0] mult_sum  = t[j] + (a_word * b_word) + carry;
    wire [35:0] mult_last = t[NUM_WORDS] + carry;
    wire [35:0] red_sum0  = t[0] + (m * p_word);
    wire [35:0] red_sum   = t[j] + (m * p_word) + carry;
    wire [35:0] red_last  = t[NUM_WORDS] + carry;

    integer k;

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= ST_IDLE;
            done   <= 1'b0;
            busy   <= 1'b0;
            result <= {WIDTH{1'b0}};
            i      <= 5'd0;
            j      <= 5'd0;
            carry  <= 36'd0;
            m      <= {WORD_SIZE{1'b0}};
            for (k = 0; k <= NUM_WORDS; k = k + 1) begin
                t[k] <= {WORD_SIZE{1'b0}};
            end
        end else begin
            case (state)
                
                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        i     <= 5'd0;
                        j     <= 5'd0;
                        carry <= 36'd0;
                        for (k = 0; k <= NUM_WORDS; k = k + 1) begin
                            t[k] <= {WORD_SIZE{1'b0}};
                        end
                        state <= ST_MULT_ACC;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // Paso 1: Acumular A[i] * B[j]
                ST_MULT_ACC: begin
                    if (j < NUM_WORDS) begin
                        t[j]  <= mult_sum[15:0];
                        carry <= mult_sum[35:16];
                        j     <= j + 1'b1;
                    end else begin
                        // j == NUM_WORDS (16)
                        t[NUM_WORDS] <= mult_last[15:0];
                        carry        <= 36'd0;
                        j            <= 5'd0;
                        state        <= ST_M_CALC;
                    end
                end

                // Paso 2: Calcular factor de reducción m
                ST_M_CALC: begin
                    m     <= t[0] * n_prime0;
                    carry <= 36'd0;
                    j     <= 5'd0;
                    state <= ST_REDUCE;
                end

                // Paso 3: Reducción m * P[j] + T[j]
                ST_REDUCE: begin
                    if (j == 5'd0) begin
                        carry <= red_sum0[35:16]; // Descarte implícito del byte bajo (Shift Right)
                        j     <= 5'd1;
                    end else if (j < NUM_WORDS) begin
                        t[j-1] <= red_sum[15:0];  // Almacenar palabra desplazada
                        carry  <= red_sum[35:16];
                        j      <= j + 1'b1;
                    end else begin
                        // j == NUM_WORDS (16)
                        t[NUM_WORDS-1] <= red_last[15:0];
                        t[NUM_WORDS]   <= red_last[31:16];
                        carry          <= 36'd0;
                        j              <= 5'd0;
                        state          <= ST_NEXT_WORD;
                    end
                end

                // Paso 4: Cambiar de palabra exterior A[i]
                ST_NEXT_WORD: begin
                    if (i == NUM_WORDS - 1) begin
                        state <= ST_FINAL_SUB;
                    end else begin
                        i     <= i + 1'b1;
                        j     <= 5'd0;
                        carry <= 36'd0;
                        state <= ST_MULT_ACC;
                    end
                end

                // Paso 5: Resta condicional T >= P
                ST_FINAL_SUB: begin
                    begin : build_res
                        reg [256:0] temp_t;
                        reg [256:0] temp_sub;
                        integer x;
                        
                        temp_t = 257'd0;
                        for (x = 0; x < NUM_WORDS; x = x + 1) begin
                            temp_t[x*WORD_SIZE +: WORD_SIZE] = t[x];
                        end
                        temp_t[256] = t[NUM_WORDS][0];

                        temp_sub = temp_t - {1'b0, p};

                        if (temp_t >= {1'b0, p}) begin
                            result <= temp_sub[255:0];
                        end else begin
                            result <= temp_t[255:0];
                        end
                    end
                    state <= ST_DONE;
                end

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