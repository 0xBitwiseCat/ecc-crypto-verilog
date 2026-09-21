module mod_add #(parameter W = 256) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [W-1:0] A,
    input wire [W-1:0] B,
    input wire [W-1:0] P,
    output reg [W-1:0] res,
    output reg done
);

    reg [W:0] sum_temp;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            res <= 0;
            done <= 0;
            state <= 0;
        end else begin
            case (state)
                0: begin // Reposo
                    done <= 0;
                    if (start) begin
                        sum_temp <= A + B; // Ciclo 1: Solo la suma
                        state <= 1;
                    end
                end
                1: begin // Ciclo 2: Comparación y reducción
                    if (sum_temp >= P) 
                        res <= sum_temp - P;
                    else 
                        res <= sum_temp[W-1:0];
                    
                    done <= 1;
                    state <= 0; // Retorno a reposo
                end
            endcase
        end
    end
endmodule