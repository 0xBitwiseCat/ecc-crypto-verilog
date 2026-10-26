// ============================================================================
// Módulo: jacobian_point_op.v (PROCESADOR MICROPROGRAMADO CON BANCO EN M9K)
// Descripción: Procesador de aritmética de puntos para secp256k1 (a = 0) en
//              dominio Montgomery. Ejecuta las rutinas:
//                CMD_INIT : Q = infinito
//                CMD_DBL  : Q = 2Q                      (Jacobiano, en sitio)
//                CMD_ADD  : Q = Q + T[tbl_idx]          (suma mixta, T afín)
//                CMD_CONV : (pub_x, pub_y) = afín normal de Q
//                           (inversión de Fermat Z^(p-2) + conversión)
// Arquitectura: todas las variables de 256 bits (tabla 1G..63G, constantes,
//               Q y temporales) viven en ecc_regfile (M9K, 1 lectura +
//               1 escritura por ciclo). Las microinstrucciones direccionan el
//               banco: los dos operandos se leen en ciclos consecutivos hacia
//               opnd_a / opnd_b, que alimentan al multiplicador de Montgomery
//               externo y a la ALU modular de 257 bits (4 etapas de media
//               palabra, para cerrar timing a 50 MHz).
// Requisitos: p - 2 debe tener el bit WIDTH-1 en 1 (cierto para secp256k1).
//             cmd y tbl_idx deben mantenerse estables mientras busy = 1.
// Optimizado para: Intel Cyclone IV (EP4CE6E22C8)
// ============================================================================

module jacobian_point_op #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    input  wire [1:0]           cmd,       // CMD_INIT / CMD_DBL / CMD_ADD / CMD_CONV
    input  wire [4:0]           tbl_idx,   // Punto de la tabla para CMD_ADD: (2*tbl_idx+1)G
    output reg                  done,
    output reg                  busy,
    input  wire [WIDTH-1:0]     p,

    // Llave pública en coordenadas afines normales (válida tras CMD_CONV)
    output reg  [WIDTH-1:0]     pub_x,
    output reg  [WIDTH-1:0]     pub_y,

    // Interfaz con el multiplicador de Montgomery
    output reg                  mult_start,
    output wire [WIDTH-1:0]     mult_a,
    output wire [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    localparam [1:0] CMD_INIT = 2'd0;
    localparam [1:0] CMD_DBL  = 2'd1;
    localparam [1:0] CMD_ADD  = 2'd2;
    localparam [1:0] CMD_CONV = 2'd3;

    // ------------------------------------------------------------------------
    // Estados del Secuenciador
    // ------------------------------------------------------------------------
    localparam [3:0] S_IDLE  = 4'd0;
    localparam [3:0] S_RA    = 4'd1;  // Decodifica; presenta la dirección de A
    localparam [3:0] S_RB    = 4'd2;  // opnd_a <= A; presenta la dirección de B
    localparam [3:0] S_LB    = 4'd3;  // opnd_b <= B; despacha MUL / ALU
    localparam [3:0] S_MWAIT = 4'd4;  // Espera al multiplicador y escribe
    localparam [3:0] S_ALU1  = 4'd5;  // s = A +/- B, mitad baja
    localparam [3:0] S_ALU2  = 4'd6;  // s = A +/- B, mitad alta
    localparam [3:0] S_ALU3  = 4'd7;  // s -/+ p, mitad baja
    localparam [3:0] S_ALU4  = 4'd8;  // s -/+ p, mitad alta y selección
    localparam [3:0] S_WB    = 4'd9;  // Escribe el resultado de la ALU

    // ------------------------------------------------------------------------
    // Formato de Microinstrucción: {op[23:21], a[20:14], b[13:7], d[6:0]}
    //   MUL/ADD/SUB: d <- a op b       BZ: if (a == 0) pc <- d
    //   BBIT: if (bit cnt de p-2 == 0) pc <- d
    //   LOOP: if (cnt != 0) {cnt <- cnt-1; pc <- d}     SETC: cnt <- WIDTH-2
    // ------------------------------------------------------------------------
    localparam [2:0] OP_MUL  = 3'd0;  // d <- a * b (Montgomery)
    localparam [2:0] OP_ADD  = 3'd1;  // d <- (a + b) mod p
    localparam [2:0] OP_SUB  = 3'd2;  // d <- (a - b) mod p
    localparam [2:0] OP_BZ   = 3'd3;
    localparam [2:0] OP_END  = 3'd4;
    localparam [2:0] OP_SETC = 3'd5;
    localparam [2:0] OP_BBIT = 3'd6;
    localparam [2:0] OP_LOOP = 3'd7;

    // Mapa del banco de registros (ver golden_model/gen_regfile.py)
    localparam [6:0] ZERO = 7'd64;    // 0
    localparam [6:0] ONE  = 7'd65;    // 1 en Montgomery (R mod p)
    localparam [6:0] LIT1 = 7'd66;    // 1 literal
    localparam [6:0] QX   = 7'd67;    // Acumulador Q (Jacobiano)
    localparam [6:0] QY   = 7'd68;
    localparam [6:0] QZ   = 7'd69;
    localparam [6:0] T0   = 7'd70;    // Temporales
    localparam [6:0] T1   = 7'd71;
    localparam [6:0] T2   = 7'd72;
    localparam [6:0] T3   = 7'd73;
    localparam [6:0] T4   = 7'd74;
    localparam [6:0] TX   = 7'd124;   // Fuente indirecta: x de T[tbl_idx]
    localparam [6:0] TY   = 7'd125;   // Fuente indirecta: y de T[tbl_idx]
    localparam [6:0] PUBX = 7'd126;   // Destino: registro de salida pub_x
    localparam [6:0] PUBY = 7'd127;   // Destino: registro de salida pub_y
    localparam [6:0] NONE = 7'd0;

    // Puntos de entrada del microprograma
    localparam [6:0] PC_INIT     = 7'd0;
    localparam [6:0] PC_END      = 7'd1;
    localparam [6:0] PC_DBL      = 7'd2;
    localparam [6:0] PC_ADD      = 7'd23;
    localparam [6:0] PC_ADD_HZ   = 7'd44;
    localparam [6:0] PC_ADD_INF  = 7'd47;
    localparam [6:0] PC_CONV     = 7'd51;
    localparam [6:0] PC_EXP_LOOP = 7'd54;
    localparam [6:0] PC_EXP_NEXT = 7'd57;
    localparam [6:0] PC_CONV_INF = 7'd65;

    // ------------------------------------------------------------------------
    // Microprograma
    // ------------------------------------------------------------------------
    function [23:0] ucode;
        input [6:0] addr;
        case (addr)
            // ===== INIT: Q = infinito (Z = 0) =====
            7'd0:  ucode = {OP_ADD,  ZERO, ZERO, QZ};
            7'd1:  ucode = {OP_END,  NONE, NONE, NONE};

            // ===== DBL: M=3XX, S=4*X*YY, X3=M^2-2S, Y3=M(S-X3)-8YYYY, Z3=2YZ
            //       (en sitio: X, Y, Z se leen antes de sobrescribirse) =====
            7'd2:  ucode = {OP_BZ,   QZ,   NONE, PC_END};  // Q = inf -> 2Q = inf
            7'd3:  ucode = {OP_MUL,  QY,   QY,   T1};      // YY
            7'd4:  ucode = {OP_MUL,  T1,   T1,   T2};      // YYYY
            7'd5:  ucode = {OP_ADD,  T2,   T2,   T2};      // 2YYYY
            7'd6:  ucode = {OP_ADD,  T2,   T2,   T2};      // 4YYYY
            7'd7:  ucode = {OP_ADD,  T2,   T2,   T2};      // 8YYYY
            7'd8:  ucode = {OP_MUL,  QX,   T1,   T1};      // X*YY
            7'd9:  ucode = {OP_ADD,  T1,   T1,   T1};      // 2*X*YY
            7'd10: ucode = {OP_ADD,  T1,   T1,   T1};      // S
            7'd11: ucode = {OP_MUL,  QX,   QX,   T0};      // XX
            7'd12: ucode = {OP_ADD,  T0,   T0,   T3};      // 2XX
            7'd13: ucode = {OP_ADD,  T0,   T3,   T0};      // M
            7'd14: ucode = {OP_MUL,  QY,   QZ,   T3};      // Y*Z
            7'd15: ucode = {OP_ADD,  T3,   T3,   QZ};      // Z3
            7'd16: ucode = {OP_MUL,  T0,   T0,   T3};      // M^2
            7'd17: ucode = {OP_SUB,  T3,   T1,   T3};      // M^2 - S
            7'd18: ucode = {OP_SUB,  T3,   T1,   QX};      // X3
            7'd19: ucode = {OP_SUB,  T1,   QX,   T3};      // S - X3
            7'd20: ucode = {OP_MUL,  T0,   T3,   T3};      // M(S - X3)
            7'd21: ucode = {OP_SUB,  T3,   T2,   QY};      // Y3
            7'd22: ucode = {OP_END,  NONE, NONE, NONE};

            // ===== ADD mixta (Z2 = 1): U2=X2*Z1^2, S2=Y2*Z1^3, H=U2-X1, R=S2-Y1,
            //       X3=R^2-H^3-2*X1*H^2, Y3=R(X1*H^2-X3)-Y1*H^3, Z3=Z1*H =====
            7'd23: ucode = {OP_BZ,   QZ,   NONE, PC_ADD_INF}; // Q = inf -> Q = T
            7'd24: ucode = {OP_MUL,  QZ,   QZ,   T0};      // Z1^2
            7'd25: ucode = {OP_MUL,  TX,   T0,   T1};      // U2
            7'd26: ucode = {OP_MUL,  QZ,   T0,   T0};      // Z1^3
            7'd27: ucode = {OP_MUL,  TY,   T0,   T0};      // S2
            7'd28: ucode = {OP_SUB,  T1,   QX,   T1};      // H
            7'd29: ucode = {OP_SUB,  T0,   QY,   T0};      // R
            7'd30: ucode = {OP_BZ,   T1,   NONE, PC_ADD_HZ}; // H == 0 -> caso especial
            7'd31: ucode = {OP_MUL,  QZ,   T1,   QZ};      // Z3 = Z1*H
            7'd32: ucode = {OP_MUL,  T1,   T1,   T2};      // H^2
            7'd33: ucode = {OP_MUL,  T2,   T1,   T3};      // H^3
            7'd34: ucode = {OP_MUL,  QX,   T2,   T2};      // V = X1*H^2
            7'd35: ucode = {OP_MUL,  T0,   T0,   T4};      // R^2
            7'd36: ucode = {OP_SUB,  T4,   T3,   T4};      // R^2 - H^3
            7'd37: ucode = {OP_SUB,  T4,   T2,   T4};      //   - V
            7'd38: ucode = {OP_SUB,  T4,   T2,   QX};      //   - V = X3
            7'd39: ucode = {OP_SUB,  T2,   QX,   T2};      // V - X3
            7'd40: ucode = {OP_MUL,  T0,   T2,   T2};      // R(V - X3)
            7'd41: ucode = {OP_MUL,  QY,   T3,   T3};      // Y1*H^3
            7'd42: ucode = {OP_SUB,  T2,   T3,   QY};      // Y3
            7'd43: ucode = {OP_END,  NONE, NONE, NONE};

            // H == 0: Q == T si además R == 0 (Q intacto -> doblar), si no Q = -T
            7'd44: ucode = {OP_BZ,   T0,   NONE, PC_DBL};  // R == 0 -> Q = 2Q
            7'd45: ucode = {OP_ADD,  ZERO, ZERO, QZ};      // Q + (-Q) = inf
            7'd46: ucode = {OP_END,  NONE, NONE, NONE};

            // Q = inf: Q = (Tx, Ty, 1)
            7'd47: ucode = {OP_ADD,  TX,   ZERO, QX};
            7'd48: ucode = {OP_ADD,  TY,   ZERO, QY};
            7'd49: ucode = {OP_ADD,  ONE,  ZERO, QZ};
            7'd50: ucode = {OP_END,  NONE, NONE, NONE};

            // ===== CONV: Z^-1 = Z^(p-2) (izquierda a derecha), x = X*Z^-2,
            //       y = Y*Z^-3, salida del dominio con MontMul(., 1) =====
            7'd51: ucode = {OP_BZ,   QZ,   NONE, PC_CONV_INF}; // Q = inf -> (0, 0)
            7'd52: ucode = {OP_ADD,  QZ,   ZERO, T0};      // acc = Z (bit WIDTH-1 de p-2)
            7'd53: ucode = {OP_SETC, NONE, NONE, NONE};    // cnt = WIDTH-2
            7'd54: ucode = {OP_MUL,  T0,   T0,   T0};      // acc = acc^2
            7'd55: ucode = {OP_BBIT, NONE, NONE, PC_EXP_NEXT}; // bit = 0 -> sin producto
            7'd56: ucode = {OP_MUL,  T0,   QZ,   T0};      // acc = acc * Z
            7'd57: ucode = {OP_LOOP, NONE, NONE, PC_EXP_LOOP};
            7'd58: ucode = {OP_MUL,  T0,   T0,   T1};      // Z^-2
            7'd59: ucode = {OP_MUL,  T1,   T0,   T2};      // Z^-3
            7'd60: ucode = {OP_MUL,  QX,   T1,   T1};      // x (Montgomery)
            7'd61: ucode = {OP_MUL,  QY,   T2,   T2};      // y (Montgomery)
            7'd62: ucode = {OP_MUL,  T1,   LIT1, PUBX};    // x normal
            7'd63: ucode = {OP_MUL,  T2,   LIT1, PUBY};    // y normal
            7'd64: ucode = {OP_END,  NONE, NONE, NONE};

            7'd65: ucode = {OP_ADD,  ZERO, ZERO, PUBX};
            7'd66: ucode = {OP_ADD,  ZERO, ZERO, PUBY};
            default: ucode = {OP_END, NONE, NONE, NONE};
        endcase
    endfunction

    // ------------------------------------------------------------------------
    // Registros
    // ------------------------------------------------------------------------
    reg  [3:0]       state;
    reg  [6:0]       pc;
    reg  [6:0]       pc_nxt;
    reg  [23:0]      ir;
    reg  [7:0]       cnt;              // Índice de bit del exponente p-2
    reg  [WIDTH:0]   alu_s;            // A +/- B (257 bits)
    reg  [WIDTH/2-1:0] adj_lo;         // Mitad baja de s -/+ p
    reg              alu_c;            // Acarreo entre mitades
    reg  [WIDTH-1:0] alu_r;            // Resultado de la ALU
    reg              alu_sub_r;
    reg  [WIDTH-1:0] opnd_a, opnd_b;   // Operandos (multiplicador y ALU)

    wire [2:0] ir_op = ir[23:21];
    wire [6:0] ir_a  = ir[20:14];
    wire [6:0] ir_b  = ir[13:7];
    wire [6:0] ir_d  = ir[6:0];

    // ------------------------------------------------------------------------
    // Banco de Registros (M9K)
    // ------------------------------------------------------------------------
    function [6:0] rf_addr;            // Resuelve las fuentes indirectas de la tabla
        input [6:0] f;
        input [4:0] idx;
        rf_addr = (f == TX) ? {1'b0, idx, 1'b0} :
                  (f == TY) ? {1'b0, idx, 1'b1} : f;
    endfunction

    wire             wb_en  = (state == S_WB) || ((state == S_MWAIT) && mult_done);
    wire             wb_pub = (ir_d == PUBX) || (ir_d == PUBY);
    wire [WIDTH-1:0] wb     = (state == S_WB) ? alu_r : mult_result;
    wire [WIDTH-1:0] rf_q;

    ecc_regfile #(.WIDTH(WIDTH), .AW(7)) rf_inst (
        .clk   (clk),
        .we    (wb_en && !wb_pub),
        .waddr (ir_d),
        .wdata (wb),
        .raddr (rf_addr((state == S_RB) ? ir_b : ir_a, tbl_idx)),
        .q     (rf_q)
    );

    wire a_is_zero = ~|rf_q;           // Válido en S_RB (rf_q = operando A)

    assign mult_a = opnd_a;
    assign mult_b = opnd_b;

    // ------------------------------------------------------------------------
    // ALU Modular: cada suma de 257 bits se parte en dos mitades con acarreo
    // registrado (una cadena de 257 bits no cierra a 50 MHz en Cyclone IV C8)
    //   S_ALU1/2: s = A + B  ó  A - B                     (257 bits)
    //   S_ALU3/4: ADD -> s - p si s >= p ; SUB -> s + p si hubo préstamo
    // ------------------------------------------------------------------------
    localparam H = WIDTH / 2;

    wire             alu_add_r = ~alu_sub_r;
    wire [WIDTH:0]   b_x       = {1'b0, opnd_b} ^ {(WIDTH+1){alu_sub_r}};
    wire [WIDTH:0]   p_x       = {1'b0, p}      ^ {(WIDTH+1){alu_add_r}};
    wire [H:0]       adj_hi    = alu_s[WIDTH:H] + p_x[WIDTH:H] + alu_c;
    wire             use_adj   = alu_sub_r ? alu_s[WIDTH] : ~adj_hi[H];

    // Exponente de Fermat: constante p - 2 (Quartus la propaga desde el top)
    wire [WIDTH-1:0] exp_p2 = p - 2'd2;
    wire             exp_bit = exp_p2[cnt];

    // ------------------------------------------------------------------------
    // Secuenciador
    // ------------------------------------------------------------------------
    always @(*) begin
        pc_nxt = pc;
        case (state)
            S_IDLE: if (start) begin
                case (cmd)
                    CMD_INIT: pc_nxt = PC_INIT;
                    CMD_DBL:  pc_nxt = PC_DBL;
                    CMD_ADD:  pc_nxt = PC_ADD;
                    default:  pc_nxt = PC_CONV;
                endcase
            end
            S_RA: case (ir_op)
                OP_SETC: pc_nxt = pc + 1'b1;
                OP_BBIT: pc_nxt = exp_bit ? pc + 1'b1 : ir_d;
                OP_LOOP: pc_nxt = (cnt != 8'd0) ? ir_d : pc + 1'b1;
                default: pc_nxt = pc;
            endcase
            S_RB:    if (ir_op == OP_BZ) pc_nxt = a_is_zero ? ir_d : pc + 1'b1;
            S_MWAIT: if (mult_done)      pc_nxt = pc + 1'b1;
            S_WB:                        pc_nxt = pc + 1'b1;
            default:                     pc_nxt = pc;
        endcase
    end

    // ROM del microprograma con salida registrada (sin reset)
    always @(posedge clk) begin
        ir <= ucode(pc_nxt);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            pc         <= PC_INIT;
            done       <= 1'b0;
            busy       <= 1'b0;
            cnt        <= 8'd0;
            alu_s      <= {(WIDTH+1){1'b0}};
            adj_lo     <= {H{1'b0}};
            alu_c      <= 1'b0;
            alu_r      <= {WIDTH{1'b0}};
            alu_sub_r  <= 1'b0;
            opnd_a     <= {WIDTH{1'b0}};
            opnd_b     <= {WIDTH{1'b0}};
            mult_start <= 1'b0;
            pub_x      <= {WIDTH{1'b0}};
            pub_y      <= {WIDTH{1'b0}};
        end else begin
            pc         <= pc_nxt;
            done       <= 1'b0;
            mult_start <= 1'b0;

            if (wb_en && ir_d == PUBX) pub_x <= wb;
            if (wb_en && ir_d == PUBY) pub_y <= wb;

            case (state)
                S_IDLE: begin
                    busy <= start;
                    if (start) state <= S_RA;
                end

                // Instrucciones sin operandos; las demás leen A
                S_RA: begin
                    case (ir_op)
                        OP_SETC: cnt <= WIDTH - 2;
                        OP_LOOP: if (cnt != 8'd0) cnt <= cnt - 1'b1;
                        OP_BBIT: ;
                        OP_END: begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_IDLE;
                        end
                        default: state <= S_RB;
                    endcase
                end

                S_RB: begin
                    opnd_a <= rf_q;
                    state  <= (ir_op == OP_BZ) ? S_RA : S_LB;
                end

                S_LB: begin
                    opnd_b <= rf_q;
                    if (ir_op == OP_MUL) begin
                        mult_start <= 1'b1;
                        state      <= S_MWAIT;
                    end else begin
                        alu_sub_r  <= (ir_op == OP_SUB);
                        state      <= S_ALU1;
                    end
                end

                S_MWAIT: if (mult_done) state <= S_RA;

                S_ALU1: begin
                    {alu_c, alu_s[H-1:0]} <= {1'b0, opnd_a[H-1:0]} + {1'b0, b_x[H-1:0]} + alu_sub_r;
                    state <= S_ALU2;
                end

                S_ALU2: begin
                    alu_s[WIDTH:H] <= {1'b0, opnd_a[WIDTH-1:H]} + b_x[WIDTH:H] + alu_c;
                    state <= S_ALU3;
                end

                S_ALU3: begin
                    {alu_c, adj_lo} <= {1'b0, alu_s[H-1:0]} + {1'b0, p_x[H-1:0]} + alu_add_r;
                    state <= S_ALU4;
                end

                S_ALU4: begin
                    alu_r <= use_adj ? {adj_hi[H-1:0], adj_lo} : alu_s[WIDTH-1:0];
                    state <= S_WB;
                end

                S_WB:    state <= S_RA;

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
