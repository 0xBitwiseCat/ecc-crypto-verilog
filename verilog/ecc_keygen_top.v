`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Top-Level Generador de Llaves ECC (secp256k1)
// OBJETIVO: Orquestar el flujo Q = k * G reutilizando hardware matemático
// ============================================================================

module ecc_keygen_top #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Entradas criptográficas
    input  wire [W-1:0] private_key_k, // Escalar aleatorio (Entropía externa)
    input  wire [W-1:0] G_X_aff,       // Punto Base X (Afín)
    input  wire [W-1:0] G_Y_aff,       // Punto Base Y (Afín)
    input  wire [W-1:0] P_MOD,         // Primo de la curva secp256k1
    
    // Salida: Llave Pública (Q)
    output reg  [W-1:0] Q_X_aff,       // Llave Pública X (Afín)
    output reg  [W-1:0] Q_Y_aff,       // Llave Pública Y (Afín)
    output reg          done
);

    // =====================================================
    // 1. SEÑALES INTERNAS Y MULTIPLEXIÓN
    // =====================================================
    
    // Estado de Control Global
    reg [2:0] global_state;
    localparam ST_IDLE       = 3'd0;
    localparam ST_PRECOMPUTE = 3'd1;
    localparam ST_SCALAR_MUL = 3'd2;
    localparam ST_CONVERT    = 3'd3;
    localparam ST_DONE       = 3'd4;

    // Bandera para saber quién controla los recursos matemáticos
    wire is_precomputing = (global_state == ST_PRECOMPUTE);
    
    // Punto G en coordenadas Jacobianas (Z = 1)
    wire [W-1:0] G_Z_jac = 256'd1; 

    // =====================================================
    // 2. INSTANCIAS DE RECURSOS COMPARTIDOS (ADD / DBL / RAM)
    // =====================================================
    
    // Entradas multiplexadas para Point Double
    wire         dbl_start = is_precomputing ? pre_dbl_start : win_dbl_start;
    wire [W-1:0] dbl_X1    = is_precomputing ? pre_dbl_X1    : win_dbl_X1;
    wire [W-1:0] dbl_Y1    = is_precomputing ? pre_dbl_Y1    : win_dbl_Y1;
    wire [W-1:0] dbl_Z1    = is_precomputing ? pre_dbl_Z1    : win_dbl_Z1;
    
    wire         dbl_done;
    wire [W-1:0] dbl_X3, dbl_Y3, dbl_Z3;

    jacobian_point_double #(.W(W)) u_shared_dbl (
        .clk(clk), .rst(rst), .start(dbl_start),
        .X1(dbl_X1), .Y1(dbl_Y1), .Z1(dbl_Z1),
        .P_MOD(P_MOD),
        .X3(dbl_X3), .Y3(dbl_Y3), .Z3(dbl_Z3),
        .done(dbl_done)
    );

    // Entradas multiplexadas para Point Add
    wire         add_start = is_precomputing ? pre_add_start : win_add_start;
    wire [W-1:0] add_X1    = is_precomputing ? pre_add_X1    : win_add_X1;
    wire [W-1:0] add_Y1    = is_precomputing ? pre_add_Y1    : win_add_Y1;
    wire [W-1:0] add_Z1    = is_precomputing ? pre_add_Z1    : win_add_Z1;
    wire [W-1:0] add_X2    = is_precomputing ? pre_add_X2    : win_add_X2;
    wire [W-1:0] add_Y2    = is_precomputing ? pre_add_Y2    : win_add_Y2;
    wire [W-1:0] add_Z2    = is_precomputing ? pre_add_Z2    : win_add_Z2;

    wire         add_done;
    wire [W-1:0] add_X3, add_Y3, add_Z3;

    jacobian_point_add #(.W(W)) u_shared_add (
        .clk(clk), .rst(rst), .start(add_start),
        .X1(add_X1), .Y1(add_Y1), .Z1(add_Z1),
        .X2(add_X2), .Y2(add_Y2), .Z2(add_Z2),
        .P_MOD(P_MOD),
        .X3(add_X3), .Y3(add_Y3), .Z3(add_Z3),
        .done(add_done)
    );

    // RAM de Doble Puerto (El precompute escribe, el window lee)
    wire         ram_we;
    wire [4:0]   ram_addr_w;
    wire [W-1:0] ram_wX, ram_wY, ram_wZ;
    wire [4:0]   ram_addr_r;
    wire [W-1:0] ram_rX, ram_rY, ram_rZ;

    ecc_point_ram #(.W(W), .ADDR_BITS(5)) u_ram (
        .clk(clk),
        .we(ram_we),
        .addr_w(ram_addr_w), .X_w(ram_wX), .Y_w(ram_wY), .Z_w(ram_wZ),
        .addr_r(ram_addr_r), .X_r(ram_rX), .Y_r(ram_rY), .Z_r(ram_rZ)
    );

    // =====================================================
    // 3. CONTROLADORES (Precompute & Window Engine)
    // =====================================================
    
    reg pre_start;
    wire pre_done;
    
    // Señales de solicitud desde precompute
    wire pre_add_start, pre_dbl_start;
    wire [W-1:0] pre_add_X1, pre_add_Y1, pre_add_Z1;
    wire [W-1:0] pre_add_X2, pre_add_Y2, pre_add_Z2;
    wire [W-1:0] pre_dbl_X1, pre_dbl_Y1, pre_dbl_Z1;

    // Nota: Deberás quitar las instancias internas de u_add y u_dbl dentro de ecc_precompute_fsm
    // y mapear sus puertos hacia estas señales.
    ecc_precompute_fsm #(.W(W)) u_precompute (
        .clk(clk), .rst(rst), .start(pre_start),
        .G_X(G_X_aff), .G_Y(G_Y_aff), .G_Z(G_Z_jac), .P_MOD(P_MOD),
        
        // Interfaz a RAM
        .ram_we(ram_we), .ram_addr(ram_addr_w),
        .ram_wX(ram_wX), .ram_wY(ram_wY), .ram_wZ(ram_wZ),
        
        // Conexión a bloques matemáticos
        .add_start(pre_add_start), .dbl_start(pre_dbl_start),
        .arg_X1(pre_add_X1), .arg_Y1(pre_add_Y1), .arg_Z1(pre_add_Z1),
        .arg_X2(pre_add_X2), .arg_Y2(pre_add_Y2), .arg_Z2(pre_add_Z2),
        .arg_dbl_X1(pre_dbl_X1), .arg_dbl_Y1(pre_dbl_Y1), .arg_dbl_Z1(pre_dbl_Z1),
        .res_add_X(add_X3), .res_add_Y(add_Y3), .res_add_Z(add_Z3),
        .res_dbl_X(dbl_X3), .res_dbl_Y(dbl_Y3), .res_dbl_Z(dbl_Z3),
        .add_done(add_done), .dbl_done(dbl_done),
        
        .precompute_done(pre_done)
    );

    reg win_start;
    wire win_done;
    wire [W-1:0] Q_X_jac, Q_Y_jac, Q_Z_jac; // Resultado final en Jacobiano
    
    // Señales de solicitud desde window engine
    wire win_add_start, win_dbl_start;
    wire [W-1:0] win_add_X1, win_add_Y1, win_add_Z1;
    wire [W-1:0] win_add_X2, win_add_Y2, win_add_Z2;
    wire [W-1:0] win_dbl_X1, win_dbl_Y1, win_dbl_Z1;

    ecc_sliding_window_engine #(.W(W)) u_window_engine (
        .clk(clk), .rst(rst), .start(win_start),
        .k(private_key_k), .P_MOD(P_MOD),
        
        // Interfaz a RAM (Lectura)
        .ram_addr_r(ram_addr_r),
        .ram_rX(ram_rX), .ram_rY(ram_rY), .ram_rZ(ram_rZ),
        
        // Conexión a bloques matemáticos
        .add_start(win_add_start), .dbl_start(win_dbl_start),
        .arg_X1(win_add_X1), .arg_Y1(win_add_Y1), .arg_Z1(win_add_Z1),
        .arg_X2(win_add_X2), .arg_Y2(win_add_Y2), .arg_Z2(win_add_Z2),
        .arg_dbl_X1(win_dbl_X1), .arg_dbl_Y1(win_dbl_Y1), .arg_dbl_Z1(win_dbl_Z1),
        .res_add_X(add_X3), .res_add_Y(add_Y3), .res_add_Z(add_Z3),
        .res_dbl_X(dbl_X3), .res_dbl_Y(dbl_Y3), .res_dbl_Z(dbl_Z3),
        .add_done(add_done), .dbl_done(dbl_done),
        
        // Resultado Final
        .res_Q_X(Q_X_jac), .res_Q_Y(Q_Y_jac), .res_Q_Z(Q_Z_jac),
        .engine_done(win_done)
    );

    // =====================================================
    // 4. CONVERSOR DE SALIDA (Jacobiano -> Afín)
    // =====================================================
    
    reg conv_start;
    wire conv_done;
    wire [W-1:0] final_Q_X_aff, final_Q_Y_aff;

    domain_coordinate_converter #(.W(W)) u_converter (
        .clk(clk), .rst(rst), .start(conv_start),
        .X_jac(Q_X_jac), .Y_jac(Q_Y_jac), .Z_jac(Q_Z_jac), .P_MOD(P_MOD),
        .X_aff(final_Q_X_aff), .Y_aff(final_Q_Y_aff),
        .done(conv_done)
    );

    // =====================================================
    // 5. MÁQUINA DE ESTADOS PRINCIPAL (Orquestador)
    // =====================================================

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            global_state <= ST_IDLE;
            pre_start <= 0;
            win_start <= 0;
            conv_start <= 0;
            done <= 0;
        end else begin
            case (global_state)
                ST_IDLE: begin
                    done <= 0;
                    if (start) begin
                        pre_start <= 1;
                        global_state <= ST_PRECOMPUTE;
                    end
                end
                
                ST_PRECOMPUTE: begin
                    pre_start <= 0;
                    if (pre_done) begin
                        win_start <= 1;
                        global_state <= ST_SCALAR_MUL;
                    end
                end
                
                ST_SCALAR_MUL: begin
                    win_start <= 0;
                    if (win_done) begin
                        conv_start <= 1;
                        global_state <= ST_CONVERT;
                    end
                end
                
                ST_CONVERT: begin
                    conv_start <= 0;
                    if (conv_done) begin
                        Q_X_aff <= final_Q_X_aff;
                        Q_Y_aff <= final_Q_Y_aff;
                        done <= 1;
                        global_state <= ST_DONE;
                    end
                end
                
                ST_DONE: begin
                    global_state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule