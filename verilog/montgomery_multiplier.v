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
    localparam ST_FINAL_SUB  = 3'b01;
    localparam ST_DONE       = 3'b110;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Registros Internos y Acumuladores
    // ------------------------------------------------------------------------
    reg [WORD_SIZE-1:0] t [0:NUM_WORDS]; // Memoria temporal T (17 palabras de 16 bits)
    reg [35:0]          acc;              // Acumulador de 36 bits
    reg [WORD_SIZE-1:0] m;                // Factor m
    
    // Índices de bucles
    reg [4:0] i; // Índice palabra exterior (0 a 15)
    reg [4:0] j; // Índice palabra interior (0 a 15)

    // Extracción de palabras de entrada
    wire [WORD_SIZE-1:0] a_word = a[i*WORD_SIZE +: WORD_SIZE];
    wire [WORD_SIZE-1:0] b_word = b[j*WORD_SIZE +: WORD_SIZE];
    wire [WORD_SIZE-1:0] p_word = p[j*WORD_SIZE +: WORD_SIZE];

    // Signals intermedias de 16 bits explícitas para eliminar el Warning (10230)
    wire [WORD_SIZE-1:0] acc_low  = acc[WORD_SIZE-1:0];
    wire [WORD_SIZE-1:0] acc_high = acc[2*WORD_SIZE-1 : WORD_SIZE]; // acc[31:16]
    wire [31:0]          m_prod   = t[0] * n_prime0;

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
            m       <= {WORD_SIZE{1'b0}};
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
                        acc   <= 36'd0;
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
                    acc  <= t[j] + (a_word * b_word) + {16'd0, acc_high};
                    t[j] <= acc_low; // Coincidencia exacta de 16 bits

                    if (j == NUM_WORDS - 1) begin
                        j     <= 5'd0;
                        state <= ST_M_CALC;
                    end else begin
                        j     <= j + 1'b1;
                    end
                end

                // Paso 2: Calcular factor de reducción m
                ST_M_CALC: begin
                    acc          <= t[NUM_WORDS] + {16'd0, acc_high};
                    t[NUM_WORDS] <= acc_low;
                    m            <= m_prod[WORD_SIZE-1:0]; // Slicing explícito a 16 bits
                    j            <= 5'd0;
                    state        <= ST_REDUCE;
                end

                // Paso 3: Reducción m * P[j] + T[j]
                ST_REDUCE: begin
                    acc <= t[j] + (m * p_word) + {16'd0, acc_high};
                    
                    if (j > 0) begin
                        t[j-1] <= acc_low; // Asignación directa wire 16-bit a reg 16-bit
                    end

                    if (j == NUM_WORDS - 1) begin
                        j     <= 5'd0;
                        state <= ST_NEXT_WORD;
                    end else begin
                        j     <= j + 1'b1;
                    end
                end

                // Paso 4: Desplazamiento final y cambio de palabra
                ST_NEXT_WORD: begin
                    acc            <= t[NUM_WORDS] + {16'd0, acc_high};
                    t[NUM_WORDS-1] <= acc_low;
                    t[NUM_WORDS]   <= acc_high; // Asignación directa 16-bit

                    if (i == NUM_WORDS - 1) begin
                        state <= ST_FINAL_SUB;
                    end else begin
                        i     <= i + 1'b1;
                        acc   <= 36'd0;
                        state <= ST_MULT_ACC;
                    end
                end

                // Paso 5: Resta condicional T >= P
                ST_FINAL_SUB: begin
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

                        if (temp_t >= p) begin
                            result <= temp_sub[WIDTH-1:0];
                        end else begin
                            result <= temp_t[WIDTH-1:0];
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