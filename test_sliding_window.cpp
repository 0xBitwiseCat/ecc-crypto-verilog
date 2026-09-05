#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>

using namespace boost::multiprecision;

// Parámetros de la curva secp256k1
const uint256_t P_MOD = uint256_t("0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F");
const uint256_t G_X = uint256_t("0x79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798");
const uint256_t G_Y = uint256_t("0x483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8");

// Estructura de punto en coordenadas Jacobianas (3 coordenadas de 32 bytes c/u = 96 bytes)
struct Point {
    uint256_t X, Y, Z;
    bool is_infinity;
    
    static Point infinity() { return {0, 0, 0, true}; }
};

// ==========================================================
// ALU Modular (Simulación del Datapath)
// ==========================================================
inline uint256_t mod_add(const uint256_t& a, const uint256_t& b) {
    return (a + b) % P_MOD;
}

inline uint256_t mod_sub(const uint256_t& a, const uint256_t& b) {
    if (a >= b) return a - b;
    return P_MOD - b + a;
}

inline uint256_t mod_mul(const uint256_t& a, const uint256_t& b) {
    return (a * b) % P_MOD;
}

// ==========================================================
// Máquina Geometría (FSM de Puntos)
// ==========================================================
Point point_double(const Point& p) {
    if (p.is_infinity || p.Y == 0) return Point::infinity();

    uint256_t Y_sq = mod_mul(p.Y, p.Y);
    uint256_t S = mod_mul(4, mod_mul(p.X, Y_sq));
    uint256_t M = mod_mul(3, mod_mul(p.X, p.X)); 

    Point r;
    r.is_infinity = false;
    r.X = mod_sub(mod_mul(M, M), mod_mul(2, S));
    r.Y = mod_sub(mod_mul(M, mod_sub(S, r.X)), mod_mul(8, mod_mul(Y_sq, Y_sq)));
    r.Z = mod_mul(2, mod_mul(p.Y, p.Z));
    return r;
}

Point point_add(const Point& p1, const Point& p2) {
    if (p1.is_infinity) return p2;
    if (p2.is_infinity) return p1;

    uint256_t Z1_sq = mod_mul(p1.Z, p1.Z);
    uint256_t Z2_sq = mod_mul(p2.Z, p2.Z);

    uint256_t U1 = mod_mul(p1.X, Z2_sq);
    uint256_t U2 = mod_mul(p2.X, Z1_sq);

    uint256_t S1 = mod_mul(p1.Y, mod_mul(p2.Z, Z2_sq));
    uint256_t S2 = mod_mul(p2.Y, mod_mul(p1.Z, Z1_sq));

    if (U1 == U2) {
        if (S1 == S2) return point_double(p1);
        return Point::infinity();
    }

    uint256_t H = mod_sub(U2, U1);
    uint256_t R = mod_sub(S2, S1);
    uint256_t H_sq = mod_mul(H, H);
    uint256_t H_cb = mod_mul(H, H_sq);

    Point r;
    r.is_infinity = false;
    r.X = mod_sub(mod_sub(mod_mul(R, R), H_cb), mod_mul(2, mod_mul(U1, H_sq)));
    r.Y = mod_sub(mod_mul(R, mod_sub(mod_mul(U1, H_sq), r.X)), mod_mul(S1, H_cb));
    r.Z = mod_mul(H, mod_mul(p1.Z, p2.Z));
    return r;
}

// ==========================================================
// Controlador Algoritmo: Ventana Deslizante (Sliding Window)
// ==========================================================
Point sliding_window_kP(uint16_t k, int w, const Point& G, const std::vector<Point>& precomp) {
    Point Q = Point::infinity();
    int i = 15; 

    while (i >= 0) {
        if (((k >> i) & 1) == 0) {
            Q = point_double(Q);
            i--;
        } else {
            int length = 1;
            int value = 1;
            int max_len = std::min(i + 1, w);

            for (int l = 1; l <= max_len; ++l) {
                if (((k >> (i - l + 1)) & 1) == 1) {
                    length = l;
                    value = (k >> (i - length + 1)) & ((1 << length) - 1);
                }
            }

            for (int j = 0; j < length; ++j) {
                Q = point_double(Q);
            }
            
            Q = point_add(Q, precomp[value / 2]);
            i -= length; 
        }
    }
    return Q;
}

// ==========================================================
// Motor de Benchmark y Generación de CSV
// ==========================================================
int main() {
    Point G = {G_X, G_Y, 1, false};
    
    std::ofstream csv("performance_results.csv");
    // Añadidas las columnas de Número Máximo y Memoria
    csv << "Window_Size,Max_Window_Value,Precomp_Points,Memory_Bytes,Total_Time_us,Avg_Time_ns_per_k\n";
    
    std::cout << "========================================================================\n";
    std::cout << " BENCHMARK: VENTANA DESLIZANTE (w = 1 a 8) para k = 1..65535\n";
    std::cout << "========================================================================\n";
    std::cout << std::left << std::setw(4) << "w" 
              << std::setw(12) << "| Max Val" 
              << std::setw(12) << "| Puntos" 
              << std::setw(15) << "| Memoria (B)" 
              << "| Avg Tiempo (ns)\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (int w = 1; w <= 8; ++w) {
        // --- 1. Cálculo de Métricas Físicas ---
        int max_window_val = (1 << w) - 1;       // 2^w - 1
        int num_precomp = 1 << (w - 1);          // 2^(w-1)
        int memory_bytes = num_precomp * 96;     // 96 bytes por punto Jacobiano (3x32B)

        // --- 2. Fase de Precomputación (Memoria) ---
        std::vector<Point> precomp(num_precomp);
        precomp[0] = G; 
        
        if (num_precomp > 1) {
            Point G2 = point_double(G); 
            for (int i = 1; i < num_precomp; ++i) {
                precomp[i] = point_add(precomp[i - 1], G2); 
            }
        }

        // --- 3. Fase de Ejecución (Test k = 1 to 65535) ---
        auto start_time = std::chrono::high_resolution_clock::now();
        uint256_t dummy_sink = 0; 

        for (uint32_t k = 1; k <= 65535; ++k) {
            Point Q = sliding_window_kP(k, w, G, precomp);
            dummy_sink ^= Q.X; // Usamos XOR para obligar al compilador a calcular Q.X
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // Operación invisible para asegurar que GCC no elimine todo el proceso por optimización (-O3)
        if (dummy_sink == 1) { std::cout << ""; }
        
        // --- 4. Métricas de Tiempo ---
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        double avg_ns = (duration_us * 1000.0) / 65535.0;

        // Escribir a CSV
        csv << w << "," 
            << max_window_val << "," 
            << num_precomp << "," 
            << memory_bytes << "," 
            << duration_us << "," 
            << std::fixed << std::setprecision(2) << avg_ns << "\n";
            
        // Imprimir en consola
        std::cout << std::left << std::setw(4) << w 
                  << "| " << std::setw(9) << max_window_val 
                  << "| " << std::setw(9) << num_precomp 
                  << "| " << std::setw(12) << memory_bytes 
                  << "| " << avg_ns << " ns\n";
    }

    csv.close();
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "Archivo 'performance_results.csv' generado exitosamente.\n";
    return 0;
}
