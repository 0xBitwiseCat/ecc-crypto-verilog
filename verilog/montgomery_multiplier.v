`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Multiplicador Modular Montgomery (Radix-2 / Serial-Bit)
// OBJETIVO: Mínimo consumo de área (0 DSPs, basado en LUTs y Flip-Flops)
// OPERACIÓN: S = (A * B * R^-1) mod P
// DOMINIO: Asume entradas pre-transformadas. Retorna salida en el mismo dominio.
// ============================================================================

module montgomery_multiplier #(
    parameter W = 256 // Ancho de bus configurado para secp256k1
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,   // Señal de inicio (pulso de 1 ciclo)
    input  wire [W-1:0] A,       // Operando A 
    input  wire [W-1:0] B,       // Operando B (Multiplicador)
    input  wire [W-1:0] P,       // Módulo Primo
    output reg  [W-1:0] S_out,   // Resultado final
    output reg          done     // Bandera de finalización (alta por 1 ciclo)
);

    // ========================================================================
    // 1. DEFINICIÓN DE ESTADOS DE LA FSM (Codificación One-Hot o Binaria)
    // Justificación: 4 estados requieren solo 2 Flip-Flops. 
    // ========================================================================
    localparam IDLE   = 2'b00; // Esperando señal de start
    localparam CALC   = 2'b01; // Ejecutando bucle de 256 iteraciones
    localparam REDUCE = 2'b10; // Resta final si S >= P
    localparam DONE_S = 2'b11; // Entregar resultado

    reg [1:0] state, next_state;

    // ========================================================================
    // 2. REGISTROS INTERNOS (Memoria del Datapath)
    // Justificación: Aislamos las entradas para que el módulo externo pueda
    // cambiar sus buses sin corromper nuestro cálculo que dura 256 ciclos.
    // ========================================================================
    reg [W-1:0] a_reg;
    reg [W-1:0] b_reg; 
    reg [W-1:0] p_reg;
    
    // IMPORTANTE: El acumulador 's_reg' necesita W+2 bits (258 bits).
    // ¿Por qué? Durante la iteración sumamos S + A + P. Si S, A y P son 
    // de 256 bits, su suma máxima es ~3P, lo que requiere 2 bits extra para
    // no perder datos (desbordamiento/overflow) antes de hacer el shift a la derecha.
    reg [W+1:0] s_reg; 

    // Contador para las iteraciones (0 a 255 requiere 8 bits. Usamos 9 para seguridad)
    reg [8:0] count;

    // ========================================================================
    // 3. LÓGICA COMBINACIONAL (La ALU interna de Montgomery)
    // Justificación: Evitamos multiplicadores. Usamos MUXes (condicionales '?:')
    // inferidos nativamente en LUTs. 
    // ========================================================================
    
    // a) Si el LSB de B es 1, preparamos A para sumar, si es 0, sumamos 0.
    wire [W+1:0] add_a = b_reg[0] ? {2'b00, a_reg} : {(W+2){1'b0}};
    
    // b) Primer sumador: S_actual + (A o 0)
    wire [W+1:0] temp_s = s_reg + add_a;
    
    // c) Evaluamos si el resultado parcial es impar (LSB == 1). 
    // Si es impar, sumamos P para volverlo par y no perder precisión al dividir.
    wire [W+1:0] add_p = temp_s[0] ? {2'b00, p_reg} : {(W+2){1'b0}};
    
    // d) Segundo sumador: temp_s + (P o 0)
    wire [W+1:0] next_s = temp_s + add_p;

    // ========================================================================
    // 4. LÓGICA SECUENCIAL: MÁQUINA DE ESTADOS FINITOS (FSM)
    // Justificación: Sincronizada por reloj (clk) para mapear a Flip-Flops.
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            a_reg <= 0;
            b_reg <= 0;
            p_reg <= 0;
            s_reg <= 0;
            count <= 0;
            S_out <= 0;
            done  <= 0;
        end else begin
            case (state)
                // ------------------------------------------------------------
                // ESTADO: IDLE
                // Espera 'start' y captura datos.
                // ------------------------------------------------------------
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        a_reg <= A;
                        b_reg <= B;
                        p_reg <= P;
                        s_reg <= 0;      // Acumulador inicia en 0
                        count <= 0;      // Contador inicia en 0
                        state <= CALC;
                    end
                end

                // ------------------------------------------------------------
                // ESTADO: CALC (Núcleo de Radix-2)
                // Justificación: Se ejecuta 256 veces. El "shift derecho" (>> 1) 
                // en next_s divide entre 2 sin consumir lógica, solo redirige cables.
                // ------------------------------------------------------------
                CALC: begin
                    // Actualiza acumulador con el resultado desplazado
                    s_reg <= next_s >> 1;
                    
                    // Desplaza el multiplicador B. 
                    // Justificación: Leer siempre b_reg[0] ahorra un gigantesco MUX 
                    // de 256 entradas que destruiría el área de la FPGA si usáramos B[count].
                    b_reg <= b_reg >> 1; 
                    
                    if (count == W - 1) begin
                        state <= REDUCE; // Si ya hicimos 256 ciclos, terminar.
                    end else begin
                        count <= count + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // ESTADO: REDUCE
                // Justificación: El algoritmo matemático dicta que el resultado
                // final puede ser mayor a P (pero siempre menor a 2P). Se hace
                // una única resta combinacional si es necesario.
                // ------------------------------------------------------------
                REDUCE: begin
                    if (s_reg >= {2'b00, p_reg}) begin
                        S_out <= s_reg[W-1:0] - p_reg; // Resta P y recorta a 256 bits
                    end else begin
                        S_out <= s_reg[W-1:0];         // Recorta y entrega
                    end
                    state <= DONE_S;
                end

                // ------------------------------------------------------------
                // ESTADO: DONE_S
                // Levanta bandera 'done' para avisar a la FSM Superior (Jacobiana).
                // ------------------------------------------------------------
                DONE_S: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule