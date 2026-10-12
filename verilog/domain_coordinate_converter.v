// ============================================================================
// Módulo: domain_coordinate_converter.v
// Descripción: Conversión de coordenadas afines (Gx, Gy) al dominio Montgomery
// Operación: Gx_mont = MontMult(Gx, R^2), Gy_mont = MontMult(Gy, R^2)
// Optimizado para: Intel Cyclone IV
// ============================================================================

module domain_coordinate_converter #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst_n,

    // Control
    input  wire         start,
    output reg          done,
    output reg          busy,

    // Entradas de dominio afín y constantes
    input  wire [W-1:0] gx_affine,
    input  wire [W-1:0] gy_affine,
    input  wire [W-1:0] p,
    input  wire [W-1:0] r2_mod_p,

    // Salidas en dominio Montgomery
    output reg  [W-1:0] gx_mont,
    output reg  [W-1:0] gy_mont,

    // Interfaz con el multiplicador de Montgomery compartido
    output reg          mult_start,
    output reg  [W-1:0] mult_a,
    output reg  [W-1:0] mult_b,
    input  wire         mult_done,
    input  wire [W-1:0] mult_result
);

    localparam ST_IDLE    = 2'b00;
    localparam ST_CALC_GX = 2'b01;
    localparam ST_CALC_GY = 2'b10;
    localparam ST_DONE    = 2'b11;

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            gx_mont    <= {W{1'b0}};
            gy_mont    <= {W{1'b0}};
            mult_start <= 1'b0;
            mult_a     <= {W{1'b0}};
            mult_b     <= {W{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        mult_a     <= gx_affine;
                        mult_b     <= r2_mod_p;
                        mult_start <= 1'b1;
                        state      <= ST_CALC_GX;
                    end else begin
                        busy       <= 1'b0;
                    end
                end

                // 1. Calcular Gx_mont = Gx * R^2 mod P
                ST_CALC_GX: begin
                    mult_start <= 1'b0;
                    if (mult_done) begin
                        gx_mont    <= mult_result;
                        mult_a     <= gy_affine;
                        mult_b     <= r2_mod_p;
                        mult_start <= 1'b1;
                        state      <= ST_CALC_GY;
                    end
                end

                // 2. Calcular Gy_mont = Gy * R^2 mod P
                ST_CALC_GY: begin
                    mult_start <= 1'b0;
                    if (mult_done) begin
                        gy_mont <= mult_result;
                        state   <= ST_DONE;
                    end
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