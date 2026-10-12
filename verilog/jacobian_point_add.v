// ============================================================================
// Módulo: jacobian_point_add.v
// Operación: P3 (X3, Y3, Z3) = P1 (X1, Y1, Z1) + P2 (X2, Y2, Z2)
// Coordenadas: Jacobianas (x = X/Z^2, y = Y/Z^3)
// ============================================================================

module jacobian_point_add #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,
    output reg                  is_double,

    // Entradas P1, P2 y Módulo P
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     x2, y2, z2,
    input  wire [WIDTH-1:0]     p,

    // Salidas P3
    output reg  [WIDTH-1:0]     x3, y3, z3,

    // Interfaz con Multiplicador de Montgomery
    output reg                  mult_start,
    output reg  [WIDTH-1:0]     mult_a,
    output reg  [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // ------------------------------------------------------------------------
    // Funciones Modulares Sintácticamente Válidas
    // ------------------------------------------------------------------------
    function [WIDTH-1:0] mod_add;
        input [WIDTH-1:0] a, b, mod;
        reg [WIDTH:0] sum;
        reg [WIDTH:0] sub;
        begin
            sum = {1'b0, a} + {1'b0, b};
            sub = sum - {1'b0, mod};
            if (sum >= {1'b0, mod})
                mod_add = sub[WIDTH-1:0];
            else
                mod_add = sum[WIDTH-1:0];
        end
    endfunction

    function [WIDTH-1:0] mod_sub;
        input [WIDTH-1:0] a, b, mod;
        reg [WIDTH:0] diff;
        begin
            if (a >= b) begin
                diff = {1'b0, a} - {1'b0, b};
            end else begin
                diff = ({1'b0, a} + {1'b0, mod}) - {1'b0, b};
            end
            mod_sub = diff[WIDTH-1:0];
        end
    endfunction

    // ------------------------------------------------------------------------
    // Codificación de Estados
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 5'd0;
    localparam ST_CHECK_ZERO = 5'd1;
    localparam ST_CALC_Z1SQ  = 5'd2;
    localparam ST_CALC_Z2SQ  = 5'd3;
    localparam ST_CALC_U1    = 5'd4;
    localparam ST_CALC_U2    = 5'd5;
    localparam ST_CALC_Z2CU  = 5'd6;
    localparam ST_CALC_Z1CU  = 5'd7;
    localparam ST_CALC_S1    = 5'd8;
    localparam ST_CALC_S2    = 5'd9;
    localparam ST_CHECK_EQ   = 5'd10;
    localparam ST_CALC_HSQ   = 5'd11;
    localparam ST_CALC_HCU   = 5'd12;
    localparam ST_CALC_V     = 5'd13;
    localparam ST_CALC_RSQ   = 5'd14;
    localparam ST_CALC_X3    = 5'd15;
    localparam ST_CALC_Y3_A  = 5'd16;
    localparam ST_CALC_Y3_B  = 5'd17;
    localparam ST_CALC_Z3_A  = 5'd18;
    localparam ST_CALC_Z3_B  = 5'd19;
    localparam ST_MULT_WAIT  = 5'd20;
    localparam ST_DONE       = 5'd21;

    reg [4:0] state, next_state;

    reg [WIDTH-1:0] z1_sq, z2_sq;
    reg [WIDTH-1:0] u1, u2;
    reg [WIDTH-1:0] s1, s2;
    reg [WIDTH-1:0] h, r;
    reg [WIDTH-1:0] h_sq, h_cu;
    reg [WIDTH-1:0] v;
    reg [WIDTH-1:0] r_sq;
    reg [WIDTH-1:0] t_y3a;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            next_state <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            is_double  <= 1'b0;
            mult_start <= 1'b0;
            mult_a     <= {WIDTH{1'b0}};
            mult_b     <= {WIDTH{1'b0}};
            x3         <= {WIDTH{1'b0}};
            y3         <= {WIDTH{1'b0}};
            z3         <= {WIDTH{1'b0}};
            z1_sq      <= {WIDTH{1'b0}};
            z2_sq      <= {WIDTH{1'b0}};
            u1         <= {WIDTH{1'b0}};
            u2         <= {WIDTH{1'b0}};
            s1         <= {WIDTH{1'b0}};
            s2         <= {WIDTH{1'b0}};
            h          <= {WIDTH{1'b0}};
            r          <= {WIDTH{1'b0}};
            h_sq       <= {WIDTH{1'b0}};
            h_cu       <= {WIDTH{1'b0}};
            v          <= {WIDTH{1'b0}};
            r_sq       <= {WIDTH{1'b0}};
            t_y3a      <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done      <= 1'b0;
                    is_double <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_ZERO;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                ST_CHECK_ZERO: begin
                    if (z1 == {WIDTH{1'b0}}) begin
                        x3    <= x2; y3 <= y2; z3 <= z2;
                        state <= ST_DONE;
                    end else if (z2 == {WIDTH{1'b0}}) begin
                        x3    <= x1; y3 <= y1; z3 <= z1;
                        state <= ST_DONE;
                    end else begin
                        mult_a     <= z1;
                        mult_b     <= z1;
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_Z1SQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                ST_CALC_Z1SQ: begin
                    z1_sq      <= mult_result;
                    mult_a     <= z2;
                    mult_b     <= z2;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z2SQ;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Z2SQ: begin
                    z2_sq      <= mult_result;
                    mult_a     <= x1;
                    mult_b     <= mult_result;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_U1;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_U1: begin
                    u1         <= mult_result;
                    mult_a     <= x2;
                    mult_b     <= z1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_U2;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_U2: begin
                    u2         <= mult_result;
                    mult_a     <= z2;
                    mult_b     <= z2_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z2CU;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Z2CU: begin
                    mult_a     <= z1;
                    mult_b     <= z1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z1CU;
                    state      <= ST_MULT_WAIT;
                    z2_sq      <= mult_result;
                end

                ST_CALC_Z1CU: begin
                    mult_a     <= y1;
                    mult_b     <= z2_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_S1;
                    state      <= ST_MULT_WAIT;
                    z1_sq      <= mult_result;
                end

                ST_CALC_S1: begin
                    s1         <= mult_result;
                    mult_a     <= y2;
                    mult_b     <= z1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_S2;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_S2: begin
                    s2    <= mult_result;
                    state <= ST_CHECK_EQ;
                end

                ST_CHECK_EQ: begin
                    h <= mod_sub(u2, u1, p);
                    r <= mod_sub(s2, s1, p);

                    if (u1 == u2) begin
                        if (s1 == s2) begin
                            is_double <= 1'b1;
                            state     <= ST_DONE;
                        end else begin
                            x3    <= {WIDTH{1'b0}};
                            y3    <= {WIDTH{1'b0}};
                            z3    <= {WIDTH{1'b0}};
                            state <= ST_DONE;
                        end
                    end else begin
                        mult_a     <= mod_sub(u2, u1, p);
                        mult_b     <= mod_sub(u2, u1, p);
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_HSQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                ST_CALC_HSQ: begin
                    h_sq       <= mult_result;
                    mult_a     <= h;
                    mult_b     <= mult_result;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_HCU;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_HCU: begin
                    h_cu       <= mult_result;
                    mult_a     <= u1;
                    mult_b     <= h_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_V;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_V: begin
                    v          <= mult_result;
                    mult_a     <= r;
                    mult_b     <= r;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_RSQ;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_RSQ: begin
                    r_sq  <= mult_result;
                    state <= ST_CALC_X3;
                end

                ST_CALC_X3: begin
                    begin
                        reg [WIDTH-1:0] temp_v2;
                        reg [WIDTH-1:0] temp_sub1;
                        temp_v2   = mod_add(v, v, p);
                        temp_sub1 = mod_sub(r_sq, h_cu, p);
                        x3        <= mod_sub(temp_sub1, temp_v2, p);
                    end

                    mult_a     <= r;
                    mult_b     <= mod_sub(v, mod_sub(mod_sub(r_sq, h_cu, p), mod_add(v, v, p), p), p);
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Y3_A;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Y3_A: begin
                    t_y3a      <= mult_result;
                    mult_a     <= s1;
                    mult_b     <= h_cu;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Y3_B;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Y3_B: begin
                    y3 <= mod_sub(t_y3a, mult_result, p);

                    mult_a     <= z1;
                    mult_b     <= z2;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z3_A;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Z3_A: begin
                    mult_a     <= mult_result;
                    mult_b     <= h;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z3_B;
                    state      <= ST_MULT_WAIT;
                end

                ST_CALC_Z3_B: begin
                    z3    <= mult_result;
                    state <= ST_DONE;
                end

                ST_MULT_WAIT: begin
                    mult_start <= 1'b0;
                    if (mult_done) begin
                        state <= next_state;
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