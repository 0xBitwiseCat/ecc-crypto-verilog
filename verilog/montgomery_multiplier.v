// ============================================================================
// Módulo: montgomery_multiplier.v (OPTIMIZADO EN ÁREA)
// Algoritmo: CIOS Montgomery Multiplication (Word-Based, W = 16 bits)
// Operación: result = (A * B * R^-1) mod P,  R = 2^WIDTH
// Arquitectura: Datapath palabra-serial con recursos únicos compartidos:
//                 - 1 multiplicador 16x16 (1 DSP 18x18) para A[i]*B[j],
//                   m*P[j] y m = T[0]*n'
//                 - 1 sumador de 32 bits:  T[j] + X*Y + carry
//                 - 1 restador de 17 bits para la resta final condicional
//               El acumulador T (NUM_WORDS+1 palabras) es un registro de
//               desplazamiento circular: se lee siempre la cabeza (T[0]) y se
//               escribe siempre la cola, sin decodificadores de escritura ni
//               mux de lectura. La resta final (T >= P ? T - P : T) se hace
//               palabra a palabra sobre el mismo anillo en dos pasadas
//               (comparar y restar), sin sumadores ni comparadores de 257 bits.
//               'result' es el propio anillo: válido cuando done = 1 y estable
//               hasta el siguiente start (los llamadores lo muestrean en done).
// Latencia: constante (independiente de los datos), 611 ciclos para 256 bits:
//           1 + NUM_WORDS*(2*NUM_WORDS+4) + 2*(NUM_WORDS+1)
// Optimizado para: Intel Cyclone IV (EP4CE6E22C8)
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

    // Entradas de datos (deben mantenerse estables mientras busy = 1)
    input  wire [WIDTH-1:0]     a,
    input  wire [WIDTH-1:0]     b,
    input  wire [WIDTH-1:0]     p,

    // Constante n_prime0 = (-P^-1) mod 2^WORD_SIZE
    input  wire [WORD_SIZE-1:0] n_prime0,

    // Salida (válida cuando done = 1)
    output wire [WIDTH-1:0]     result
);

    localparam W  = WORD_SIZE;
    localparam TW = (NUM_WORDS + 1) * W;          // Ancho del anillo T
    localparam IW = $clog2(NUM_WORDS);            // Índice de palabra (0..15)
    localparam JW = $clog2(NUM_WORDS + 2);        // Contador de paso (0..17)

    // ------------------------------------------------------------------------
    // Estados de la FSM
    // ------------------------------------------------------------------------
    localparam [2:0] ST_IDLE     = 3'd0;
    localparam [2:0] ST_MULT_ACC = 3'd1;  // T += A[i] * B          (17 pasos)
    localparam [2:0] ST_M_CALC   = 3'd2;  // m = T[0] * n' mod W    (1 paso)
    localparam [2:0] ST_REDUCE   = 3'd3;  // T = (T + m * P) / W    (18 pasos)
    localparam [2:0] ST_CMP      = 3'd4;  // borrow de T - P        (17 pasos)
    localparam [2:0] ST_SUB      = 3'd5;  // T = T - P si T >= P    (17 pasos)

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Registros Internos
    // ------------------------------------------------------------------------
    reg [TW-1:0] t;       // Anillo T: t[W-1:0] = cabeza (T[0])
    reg [W-1:0]  carry;   // Acarreo entre palabras (máximo W-1)
    reg [W-1:0]  m;       // Factor de reducción
    reg          borrow;  // Préstamo de la resta serial T - P
    reg          t_ge_p;  // Resultado de la comparación T >= P
    reg [IW-1:0] i;       // Palabra exterior A[i]
    reg [JW-1:0] j;       // Paso dentro de la fase

    // ------------------------------------------------------------------------
    // Selección de Palabras (índices de IW bits: sin part-selects fuera de rango)
    // ------------------------------------------------------------------------
    wire          j_in   = (j < NUM_WORDS);
    wire [W-1:0]  t_head = t[W-1:0];
    wire [W-1:0]  a_word = a[i * W +: W];
    wire [W-1:0]  b_word = b[j[IW-1:0] * W +: W];
    wire [W-1:0]  p_word = j_in ? p[j[IW-1:0] * W +: W] : {W{1'b0}};

    // ------------------------------------------------------------------------
    // Multiplicador Único Compartido
    // ------------------------------------------------------------------------
    wire [W-1:0]   mul_x = (state == ST_M_CALC) ? t_head :
                           (state == ST_REDUCE) ? m      : a_word;
    wire [W-1:0]   mul_y = (state == ST_M_CALC) ? n_prime0     :
                           !j_in                ? {W{1'b0}}    :
                           (state == ST_REDUCE) ? p_word       : b_word;
    wire [2*W-1:0] prod  = mul_x * mul_y;

    // ------------------------------------------------------------------------
    // Sumador Único: T[j] + X*Y + carry  (máximo W^2 - 1: cabe en 2W bits)
    //   El bit T[s+1] no tiene registro propio: sale de la multiplicación en
    //   'carry' y entra como acarreo al paso j = 0 de la reducción, donde
    //   T[0] + m*P[0] = k*W exacto, así que la palabra baja vale T[s+1] y
    //   viaja por el anillo hasta el paso j = s+1.
    // ------------------------------------------------------------------------
    wire [2*W-1:0] sum = prod + t_head + carry;

    // ------------------------------------------------------------------------
    // Restador Serial de la Resta Final
    // ------------------------------------------------------------------------
    wire [W:0] diff = {1'b0, t_head} - {1'b0, p_word} - borrow;

    // ------------------------------------------------------------------------
    // Anillo T: entra por la cola, rota una palabra por ciclo
    // ------------------------------------------------------------------------
    wire         t_shift = (state != ST_IDLE) && (state != ST_M_CALC);
    wire [W-1:0] t_tail  = (state == ST_SUB) ? (t_ge_p ? diff[W-1:0] : t_head) :
                           (state == ST_CMP) ? t_head : sum[W-1:0];

    assign result = t[WIDTH-1:0];

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= ST_IDLE;
            done   <= 1'b0;
            busy   <= 1'b0;
            t      <= {TW{1'b0}};
            carry  <= {W{1'b0}};
            m      <= {W{1'b0}};
            borrow <= 1'b0;
            t_ge_p <= 1'b0;
            i      <= {IW{1'b0}};
            j      <= {JW{1'b0}};
        end else begin
            done <= 1'b0;

            if (state == ST_IDLE && start) t <= {TW{1'b0}};          // T = 0
            else if (t_shift)              t <= {t_tail, t[TW-1:W]}; // Rotación

            case (state)
                ST_IDLE: begin
                    busy   <= start;
                    i      <= {IW{1'b0}};
                    j      <= {JW{1'b0}};
                    carry  <= {W{1'b0}};
                    borrow <= 1'b0;
                    if (start) state <= ST_MULT_ACC;
                end

                // Paso 1: (carry, T[j]) = T[j] + A[i]*B[j] + carry
                //         j = s: (T[s+1], T[s]) = T[s] + carry
                ST_MULT_ACC: begin
                    carry <= sum[2*W-1:W];
                    if (j == NUM_WORDS) begin
                        j     <= {JW{1'b0}};
                        state <= ST_M_CALC;
                    end else begin
                        j     <= j + 1'b1;
                    end
                end

                // Paso 2: m = T[0] * n' mod W (el anillo no rota)
                ST_M_CALC: begin
                    m     <= prod[W-1:0];
                    state <= ST_REDUCE;
                end

                // Paso 3: (carry, T[j-1]) = T[j] + m*P[j] + carry
                //         j = 0 escribe T[s+1] (palabra baja de T[0] + m*P[0] + T[s+1]),
                //         j = s+1 lo lee y escribe T[s] = T[s+1] + carry
                ST_REDUCE: begin
                    if (j == NUM_WORDS + 1) begin
                        carry <= {W{1'b0}};
                        j     <= {JW{1'b0}};
                        i     <= i + 1'b1;
                        state <= (i == NUM_WORDS - 1) ? ST_CMP : ST_MULT_ACC;
                    end else begin
                        carry <= sum[2*W-1:W];
                        j     <= j + 1'b1;
                    end
                end

                // Paso 4: T >= P  <=>  T - P no produce préstamo (incluye T[s])
                ST_CMP: begin
                    if (j == NUM_WORDS) begin
                        t_ge_p <= ~diff[W];
                        borrow <= 1'b0;
                        j      <= {JW{1'b0}};
                        state  <= ST_SUB;
                    end else begin
                        borrow <= diff[W];
                        j      <= j + 1'b1;
                    end
                end

                // Paso 5: T = T - P si T >= P (siempre se recorre: tiempo constante)
                ST_SUB: begin
                    borrow <= diff[W];
                    if (j == NUM_WORDS) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        j <= j + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
