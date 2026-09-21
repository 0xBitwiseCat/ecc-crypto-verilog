module mod_sub #(parameter W = 256) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [W-1:0] A,
    input wire [W-1:0] B,
    input wire [W-1:0] P,
    output reg [W-1:0] res,
    output reg done
);

    reg [W:0] diff_reg; // Vector de W+1 bits para capturar el bit de signo/borrow
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            res <= 0;
            done <= 0;
            state <= 0;
            diff_reg <= 0;
        end else begin
            case (state)
                0: begin // Reposo
                    done <= 0;
                    if (start) begin
                        // Ciclo 1: Resta agregando un bit '0' a la izquierda para evitar truncamiento
                        diff_reg <= {1'b0, A} - {1'b0, B}; 
                        state <= 1;
                    end
                end
                1: begin // Ciclo 2: Reducción y corrección
                    // Verificamos el MSB (diff_reg[W]). Si es 1, hubo underflow (A < B).
                    if (diff_reg[W]) 
                        res <= diff_reg[W-1:0] + P; // Corrección sumando el módulo
                    else 
                        res <= diff_reg[W-1:0];     // El resultado ya es correcto
                    
                    done <= 1;
                    state <= 0; // Retorno a reposo
                end
                default: state <= 0;
            endcase
        end
    end
endmodule