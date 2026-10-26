// ============================================================================
// Módulo: jacobian_point_op.v (MICROPROGRAMADO)
// Descripción: Operador Geométrico-Jacobiano Unificado (Adición y Doblamiento)
//              para secp256k1 (a=0) en Dominio Montgomery.
// Arquitectura: Secuenciador microprogramado (6 estados + PC + ROM) sobre un
//               datapath único de dos buses de operandos:
//                 - Bus A: entradas (x1,y1,z1,x2,y2,z2), acumulador ACC o 0
//                 - Bus B: temporales (T0,T1), salidas (X3,Y3,Z3), copia de A o 0
//               Las salidas x3/y3/z3 se reutilizan como temporales: los
//               llamadores sólo las muestrean cuando 'done' = 1.
// ALU Modular: 3 ciclos, un solo sumador de 257 bits por etapa.
// Optimizado para: Intel Cyclone IV (EP4CE6E22C8)
// ============================================================================

module jacobian_point_op #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    input  wire                 op_type,   // 0: Addition (P + Q), 1: Doubling (2P)
    output reg                  done,
    output reg                  busy,
    output reg                  is_double, // Detecta P == Q durante Adición

    // Coordenadas Jacobianas de Entrada (Montgomery)
    // Deben mantenerse estables mientras busy = 1
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     x2, y2, z2, // Requerido solo para op_type = 0
    input  wire [WIDTH-1:0]     p,

    // Coordenadas Jacobianas de Salida (válidas cuando done = 1)
    output reg  [WIDTH-1:0]     x3, y3, z3,

    // Interfaz con el multiplicador de Montgomery compartido
    output wire                 mult_start,
    output wire [WIDTH-1:0]     mult_a,
    output wire [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // ------------------------------------------------------------------------
    // Estados del Secuenciador
    // ------------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_EXEC  = 3'd1;  // Decodifica, registra operandos, salta
    localparam [2:0] S_MWAIT = 3'd2;  // Espera al multiplicador y escribe
    localparam [2:0] S_ALU1  = 3'd3;  // Etapa 1 de la ALU (suma/resta)
    localparam [2:0] S_AWB   = 3'd4;  // Etapa 2 de la ALU (corrección mod p)
    localparam [2:0] S_WB    = 3'd5;  // Escribe el resultado de la ALU

    // ------------------------------------------------------------------------
    // Formato de Microinstrucción: {op[14:12], a[11:9], b[8:6], dst[5:0]}
    //   OP_BZ : dst = dirección de salto si A == 0
    //   OP_INF: b[0] = 1 -> is_double <= (A == 0)
    // ------------------------------------------------------------------------
    localparam [2:0] OP_MUL = 3'd0;  // dst <- A * B (Montgomery)
    localparam [2:0] OP_ADD = 3'd1;  // dst <- (A + B) mod p
    localparam [2:0] OP_SUB = 3'd2;  // dst <- (A - B) mod p
    localparam [2:0] OP_BZ  = 3'd3;  // if (A == 0) pc <- dst
    localparam [2:0] OP_INF = 3'd4;  // (x3,y3,z3) <- 0, fin
    localparam [2:0] OP_END = 3'd5;  // fin

    // Fuentes del bus A
    localparam [2:0] A_X1  = 3'd0;
    localparam [2:0] A_Y1  = 3'd1;
    localparam [2:0] A_Z1  = 3'd2;
    localparam [2:0] A_X2  = 3'd3;
    localparam [2:0] A_Y2  = 3'd4;
    localparam [2:0] A_Z2  = 3'd5;
    localparam [2:0] A_ACC = 3'd6;
    localparam [2:0] A_0   = 3'd7;

    // Fuentes del bus B
    localparam [2:0] B_T0  = 3'd0;
    localparam [2:0] B_T1  = 3'd1;
    localparam [2:0] B_X3  = 3'd2;
    localparam [2:0] B_Y3  = 3'd3;
    localparam [2:0] B_Z3  = 3'd4;
    localparam [2:0] B_CPY = 3'd5;   // B = A (cuadrados y duplicaciones)
    localparam [2:0] B_0   = 3'd6;

    // Máscara de destinos (se permiten escrituras múltiples)
    localparam [5:0] D_NONE = 6'b000000;
    localparam [5:0] D_ACC  = 6'b000001;
    localparam [5:0] D_T0   = 6'b000010;
    localparam [5:0] D_T1   = 6'b000100;
    localparam [5:0] D_X3   = 6'b001000;
    localparam [5:0] D_Y3   = 6'b010000;
    localparam [5:0] D_Z3   = 6'b100000;

    // Puntos de entrada del microprograma
    localparam [5:0] PC_DBL  = 6'd0;
    localparam [5:0] PC_ADD  = 6'd22;
    localparam [5:0] PC_HZ   = 6'd51;
    localparam [5:0] PC_INF  = 6'd53;
    localparam [5:0] PC_RET2 = 6'd54;
    localparam [5:0] PC_RET1 = 6'd58;

    // ------------------------------------------------------------------------
    // Microprograma
    // ------------------------------------------------------------------------
    function [14:0] ucode;
        input [5:0] addr;
        case (addr)
            // ===== DOBLAMIENTO: M=3XX, S=4*X1*YY, X3=M^2-2S,
            //       Y3=M(S-X3)-8YYYY, Z3=2*Y1*Z1 =====
            6'd0:  ucode = {OP_BZ,  A_Z1,  B_0,   PC_INF};      // Z1 == 0 -> infinito
            6'd1:  ucode = {OP_MUL, A_Y1,  B_CPY, D_ACC|D_T1};  // YY
            6'd2:  ucode = {OP_MUL, A_ACC, B_CPY, D_ACC};       // YYYY
            6'd3:  ucode = {OP_ADD, A_ACC, B_CPY, D_ACC};       // 2YYYY
            6'd4:  ucode = {OP_ADD, A_ACC, B_CPY, D_ACC};       // 4YYYY
            6'd5:  ucode = {OP_ADD, A_ACC, B_CPY, D_Z3};        // 8YYYY
            6'd6:  ucode = {OP_MUL, A_X1,  B_T1,  D_ACC};       // X1*YY
            6'd7:  ucode = {OP_ADD, A_ACC, B_CPY, D_ACC};       // 2*X1*YY
            6'd8:  ucode = {OP_ADD, A_ACC, B_CPY, D_T1};        // S
            6'd9:  ucode = {OP_MUL, A_X1,  B_CPY, D_ACC};       // XX
            6'd10: ucode = {OP_ADD, A_ACC, B_CPY, D_T0};        // 2XX
            6'd11: ucode = {OP_ADD, A_ACC, B_T0,  D_ACC|D_T0};  // M
            6'd12: ucode = {OP_MUL, A_ACC, B_CPY, D_ACC};       // M^2
            6'd13: ucode = {OP_SUB, A_ACC, B_T1,  D_ACC};       // M^2 - S
            6'd14: ucode = {OP_SUB, A_ACC, B_T1,  D_X3};        // X3
            6'd15: ucode = {OP_ADD, A_0,   B_T1,  D_ACC};       // ACC <- S
            6'd16: ucode = {OP_SUB, A_ACC, B_X3,  D_ACC};       // S - X3
            6'd17: ucode = {OP_MUL, A_ACC, B_T0,  D_ACC};       // M(S - X3)
            6'd18: ucode = {OP_SUB, A_ACC, B_Z3,  D_Y3};        // Y3
            6'd19: ucode = {OP_ADD, A_Y1,  B_CPY, D_T0};        // 2Y1
            6'd20: ucode = {OP_MUL, A_Z1,  B_T0,  D_Z3};        // Z3
            6'd21: ucode = {OP_END, A_0,   B_0,   D_NONE};

            // ===== ADICIÓN: U1=X1*Z2^2, U2=X2*Z1^2, S1=Y1*Z2^3, S2=Y2*Z1^3,
            //       H=U2-U1, R=S2-S1, X3=R^2-H^3-2*U1*H^2,
            //       Y3=R(U1*H^2-X3)-S1*H^3, Z3=Z1*Z2*H =====
            6'd22: ucode = {OP_BZ,  A_Z1,  B_0,   PC_RET2};     // Z1 == 0 -> P2
            6'd23: ucode = {OP_BZ,  A_Z2,  B_0,   PC_RET1};     // Z2 == 0 -> P1
            6'd24: ucode = {OP_MUL, A_Z2,  B_CPY, D_T0};        // Z2^2
            6'd25: ucode = {OP_MUL, A_X1,  B_T0,  D_T1};        // U1
            6'd26: ucode = {OP_MUL, A_Z2,  B_T0,  D_T0};        // Z2^3
            6'd27: ucode = {OP_MUL, A_Y1,  B_T0,  D_T0};        // S1
            6'd28: ucode = {OP_MUL, A_Z1,  B_CPY, D_X3};        // Z1^2
            6'd29: ucode = {OP_MUL, A_Z1,  B_X3,  D_Y3};        // Z1^3
            6'd30: ucode = {OP_MUL, A_Y2,  B_Y3,  D_ACC};       // S2
            6'd31: ucode = {OP_SUB, A_ACC, B_T0,  D_Y3};        // R
            6'd32: ucode = {OP_MUL, A_X2,  B_X3,  D_ACC};       // U2
            6'd33: ucode = {OP_SUB, A_ACC, B_T1,  D_ACC|D_X3};  // H
            6'd34: ucode = {OP_BZ,  A_ACC, B_0,   PC_HZ};       // H == 0 -> caso especial
            6'd35: ucode = {OP_MUL, A_Z1,  B_X3,  D_Z3};        // Z1*H
            6'd36: ucode = {OP_MUL, A_Z2,  B_Z3,  D_Z3};        // Z3
            6'd37: ucode = {OP_MUL, A_ACC, B_CPY, D_ACC};       // H^2
            6'd38: ucode = {OP_MUL, A_ACC, B_T1,  D_T1};        // V = U1*H^2
            6'd39: ucode = {OP_MUL, A_ACC, B_X3,  D_ACC|D_X3};  // H^3
            6'd40: ucode = {OP_MUL, A_ACC, B_T0,  D_T0};        // S1*H^3
            6'd41: ucode = {OP_ADD, A_0,   B_Y3,  D_ACC};       // ACC <- R
            6'd42: ucode = {OP_MUL, A_ACC, B_CPY, D_ACC};       // R^2
            6'd43: ucode = {OP_SUB, A_ACC, B_X3,  D_ACC};       // R^2 - H^3
            6'd44: ucode = {OP_SUB, A_ACC, B_T1,  D_ACC};       //   - V
            6'd45: ucode = {OP_SUB, A_ACC, B_T1,  D_X3};        //   - V = X3
            6'd46: ucode = {OP_ADD, A_0,   B_T1,  D_ACC};       // ACC <- V
            6'd47: ucode = {OP_SUB, A_ACC, B_X3,  D_ACC};       // V - X3
            6'd48: ucode = {OP_MUL, A_ACC, B_Y3,  D_ACC};       // R(V - X3)
            6'd49: ucode = {OP_SUB, A_ACC, B_T0,  D_Y3};        // Y3
            6'd50: ucode = {OP_END, A_0,   B_0,   D_NONE};

            // H == 0 (ACC = 0, Y3 = R): P == Q si además R == 0
            6'd51: ucode = {OP_ADD, A_ACC, B_Y3,  D_ACC};       // ACC <- R
            6'd52: ucode = {OP_INF, A_ACC, 3'd1,  D_NONE};      // is_double <= (R == 0)

            // Resultado en el infinito
            6'd53: ucode = {OP_INF, A_0,   3'd0,  D_NONE};

            // Z1 == 0: resultado = P2
            6'd54: ucode = {OP_ADD, A_X2,  B_0,   D_X3};
            6'd55: ucode = {OP_ADD, A_Y2,  B_0,   D_Y3};
            6'd56: ucode = {OP_ADD, A_Z2,  B_0,   D_Z3};
            6'd57: ucode = {OP_END, A_0,   B_0,   D_NONE};

            // Z2 == 0: resultado = P1
            6'd58: ucode = {OP_ADD, A_X1,  B_0,   D_X3};
            6'd59: ucode = {OP_ADD, A_Y1,  B_0,   D_Y3};
            6'd60: ucode = {OP_ADD, A_Z1,  B_0,   D_Z3};
            default: ucode = {OP_END, A_0, B_0,   D_NONE};
        endcase
    endfunction

    // ------------------------------------------------------------------------
    // Registros
    // ------------------------------------------------------------------------
    reg  [2:0]       state;
    reg  [5:0]       pc;
    reg  [5:0]       pc_nxt;
    reg  [14:0]      ir;
    reg  [WIDTH-1:0] acc, t0, t1;
    reg  [WIDTH:0]   alu_s;            // Etapa 1 de la ALU
    reg  [WIDTH-1:0] alu_r;            // Resultado de la ALU
    reg  [WIDTH-1:0] opnd_a, opnd_b;   // Operandos registrados (multiplicador y ALU)
    reg              mult_start_r;

    wire [2:0] ir_op  = ir[14:12];
    wire [2:0] ir_a   = ir[11:9];
    wire [2:0] ir_b   = ir[8:6];
    wire [5:0] ir_dst = ir[5:0];

    // ------------------------------------------------------------------------
    // Buses de Operandos
    // ------------------------------------------------------------------------
    reg [WIDTH-1:0] op_a, op_b;

    always @(*) begin
        case (ir_a)
            A_X1:    op_a = x1;
            A_Y1:    op_a = y1;
            A_Z1:    op_a = z1;
            A_X2:    op_a = x2;
            A_Y2:    op_a = y2;
            A_Z2:    op_a = z2;
            A_ACC:   op_a = acc;
            default: op_a = {WIDTH{1'b0}};
        endcase

        case (ir_b)
            B_T0:    op_b = t0;
            B_T1:    op_b = t1;
            B_X3:    op_b = x3;
            B_Y3:    op_b = y3;
            B_Z3:    op_b = z3;
            B_CPY:   op_b = op_a;
            default: op_b = {WIDTH{1'b0}};
        endcase
    end

    wire a_is_zero = ~|op_a;

    assign mult_start = mult_start_r;
    assign mult_a     = opnd_a;
    assign mult_b     = opnd_b;

    // ------------------------------------------------------------------------
    // ALU Modular de 2 Etapas + registro de resultado
    //   Etapa 1 (S_ALU1): s = A + B  ó  A - B           (257 bits)
    //   Etapa 2 (S_AWB) : ADD -> s - p si s >= p
    //                     SUB -> s + p si hubo préstamo
    // ------------------------------------------------------------------------
    wire alu_sub = (ir_op == OP_SUB);
    reg  alu_sub_r;                    // Operación registrada: las etapas no dependen de ir
    wire alu_add_r = ~alu_sub_r;

    wire [WIDTH:0]   alu_adj     = alu_s + ({1'b0, p} ^ {(WIDTH+1){alu_add_r}}) + alu_add_r;
    wire             alu_use_adj = alu_sub_r ? alu_s[WIDTH] : ~alu_adj[WIDTH];
    wire [WIDTH-1:0] alu_res     = alu_use_adj ? alu_adj[WIDTH-1:0] : alu_s[WIDTH-1:0];

    // ------------------------------------------------------------------------
    // Escritura de Resultados
    // ------------------------------------------------------------------------
    wire             wb_en = (state == S_WB) || ((state == S_MWAIT) && mult_done);
    wire [WIDTH-1:0] wb    = (state == S_WB) ? alu_r : mult_result;

    // ------------------------------------------------------------------------
    // Secuenciador
    // ------------------------------------------------------------------------
    always @(*) begin
        pc_nxt = pc;
        case (state)
            S_IDLE:  if (start)            pc_nxt = op_type ? PC_DBL : PC_ADD;
            S_EXEC:  if (ir_op == OP_BZ)   pc_nxt = a_is_zero ? ir_dst : pc + 1'b1;
            S_MWAIT: if (mult_done)        pc_nxt = pc + 1'b1;
            S_WB:                          pc_nxt = pc + 1'b1;
            default:                       pc_nxt = pc;
        endcase
    end

    // ROM con salida registrada (sin reset)
    always @(posedge clk) begin
        ir <= ucode(pc_nxt);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            pc        <= PC_DBL;
            done      <= 1'b0;
            busy      <= 1'b0;
            is_double <= 1'b0;
            acc       <= {WIDTH{1'b0}};
            t0        <= {WIDTH{1'b0}};
            t1        <= {WIDTH{1'b0}};
            x3        <= {WIDTH{1'b0}};
            y3        <= {WIDTH{1'b0}};
            z3        <= {WIDTH{1'b0}};
            alu_s     <= {(WIDTH+1){1'b0}};
            alu_sub_r <= 1'b0;
            alu_r     <= {WIDTH{1'b0}};
            opnd_a    <= {WIDTH{1'b0}};
            opnd_b    <= {WIDTH{1'b0}};
            mult_start_r <= 1'b0;
        end else begin
            pc           <= pc_nxt;
            done         <= 1'b0;
            mult_start_r <= 1'b0;

            if (wb_en) begin
                if (ir_dst[0]) acc <= wb;
                if (ir_dst[1]) t0  <= wb;
                if (ir_dst[2]) t1  <= wb;
                if (ir_dst[3]) x3  <= wb;
                if (ir_dst[4]) y3  <= wb;
                if (ir_dst[5]) z3  <= wb;
            end

            case (state)
                S_IDLE: begin
                    is_double <= 1'b0;
                    busy      <= start;
                    if (start) state <= S_EXEC;
                end

                S_EXEC: begin
                    opnd_a <= op_a;
                    opnd_b <= op_b;
                    case (ir_op)
                        OP_MUL: begin
                            mult_start_r <= 1'b1;
                            state        <= S_MWAIT;
                        end

                        OP_ADD, OP_SUB: begin
                            alu_sub_r <= alu_sub;
                            state     <= S_ALU1;
                        end

                        OP_BZ: state <= S_EXEC;

                        OP_INF: begin
                            x3        <= {WIDTH{1'b0}};
                            y3        <= {WIDTH{1'b0}};
                            z3        <= {WIDTH{1'b0}};
                            is_double <= ir_b[0] & a_is_zero;
                            done      <= 1'b1;
                            busy      <= 1'b0;
                            state     <= S_IDLE;
                        end

                        default: begin // OP_END
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_IDLE;
                        end
                    endcase
                end

                S_MWAIT: if (mult_done) state <= S_EXEC;

                S_ALU1: begin
                    alu_s <= {1'b0, opnd_a} + ({1'b0, opnd_b} ^ {(WIDTH+1){alu_sub_r}}) + alu_sub_r;
                    state <= S_AWB;
                end

                S_AWB: begin
                    alu_r <= alu_res;
                    state <= S_WB;
                end

                S_WB:    state <= S_EXEC;

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
