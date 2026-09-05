# ==========================================
# CAPA 4: ARITMÉTICA DE ENTEROS (KARATSUBA)
# ==========================================
def karatsuba(x, y):
    """Multiplica dos enteros grandes usando la técnica de divide y vencerás."""
    if x < 10 or y < 10:
        return x * y
    
    # Calcula el tamaño de los números
    n = max(len(str(x)), len(str(y)))
    m = n // 2
    
    # Divide x y y en mitades
    high1, low1 = divmod(x, 10**m)
    high2, low2 = divmod(y, 10**m)
    
    # 3 multiplicaciones recursivas
    z0 = karatsuba(low1, low2)
    z1 = karatsuba((low1 + high1), (low2 + high2))
    z2 = karatsuba(high1, high2)
    
    # Ensambla el resultado
    return (z2 * 10**(2 * m)) + ((z1 - z2 - z0) * 10**m) + z0

# ==========================================
# CAPA 3: ARITMÉTICA MODULAR
# ==========================================
# Parámetros de la curva secp256k1
P = 2**256 - 2**32 - 2**9 - 2**8 - 2**7 - 2**6 - 2**4 - 1
A = 0 # En secp256k1, a = 0

# Coordenadas Afines del Punto Generador G
G_X = 0x79be667ef9dcbbac55a06295ce870b07029bfcdb2dce28d959f2815b16f81798
G_Y = 0x483ada7726a3c4655da4fbfc0e1108a8fd17b448a68554199c47d08ffb10d4b8

def mod_mult(a, b):
    """Multiplicación en el cuerpo finito usando nuestro Karatsuba."""
    return karatsuba(a, b) % P

def mod_inv(n):
    """Inversión modular usando el Pequeño Teorema de Fermat (n^(P-2) mod P).
       Esta es la operación costosa que intentamos evitar con las Jacobianas."""
    return pow(n, P - 2, P)

# ==========================================
# CAPA 2: ARITMÉTICA DE PUNTOS (JACOBIANAS)
# ==========================================
# Un punto Jacobiano se representa como la tupla (X, Y, Z)
# El punto afín correspondiente es (X / Z^2, Y / Z^3)

def jacobian_double(point):
    """Dobla un punto (2P) usando coordenadas Jacobianas (sin divisiones)."""
    X1, Y1, Z1 = point
    if Y1 == 0 or Z1 == 0:
        return (0, 0, 0) # Punto en el infinito
    
    # A = Y1^2 mod P
    A = mod_mult(Y1, Y1)
    # B = 4 * X1 * A mod P
    B = mod_mult(4 * X1, A)
    # C = 8 * A^2 mod P
    C = mod_mult(8, mod_mult(A, A))
    # D = 3 * X1^2 mod P (porque a=0 en secp256k1)
    D = mod_mult(3, mod_mult(X1, X1))
    
    # X3 = D^2 - 2B
    X3 = (mod_mult(D, D) - 2 * B) % P
    # Y3 = D(B - X3) - C
    Y3 = (mod_mult(D, (B - X3)) - C) % P
    # Z3 = 2 * Y1 * Z1
    Z3 = mod_mult(2 * Y1, Z1)
    
    return (X3, Y3, Z3)

def jacobian_add(p1, p2):
    """Suma dos puntos (P1 + P2) usando coordenadas Jacobianas (sin divisiones)."""
    X1, Y1, Z1 = p1
    X2, Y2, Z2 = p2
    
    # Manejo del punto en el infinito
    if Z1 == 0: return p2
    if Z2 == 0: return p1
    
    # Z1_sq = Z1^2, Z2_sq = Z2^2
    Z1_sq = mod_mult(Z1, Z1)
    Z2_sq = mod_mult(Z2, Z2)
    
    # U1 = X1 * Z2^2, U2 = X2 * Z1^2
    U1 = mod_mult(X1, Z2_sq)
    U2 = mod_mult(X2, Z1_sq)
    
    # S1 = Y1 * Z2^3, S2 = Y2 * Z1^3
    S1 = mod_mult(Y1, mod_mult(Z2_sq, Z2))
    S2 = mod_mult(Y2, mod_mult(Z1_sq, Z1))
    
    if U1 == U2:
        if S1 == S2:
            return jacobian_double(p1)
        return (0, 0, 0) # P + (-P) = Infinito
    
    H = (U2 - U1) % P
    R = (S2 - S1) % P
    
    H_sq = mod_mult(H, H)
    H_cb = mod_mult(H_sq, H)
    
    # X3 = R^2 - H^3 - 2 * U1 * H^2
    X3 = (mod_mult(R, R) - H_cb - mod_mult(2 * U1, H_sq)) % P
    
    # Y3 = R * (U1 * H^2 - X3) - S1 * H^3
    Y3 = (mod_mult(R, (mod_mult(U1, H_sq) - X3)) - mod_mult(S1, H_cb)) % P
    
    # Z3 = H * Z1 * Z2
    Z3 = mod_mult(H, mod_mult(Z1, Z2))
    
    return (X3, Y3, Z3)

# ==========================================
# CAPA 1: MULTIPLICACIÓN ESCALAR 
# ==========================================
def double_and_add(k, point):
    """Calcula k * P usando el algoritmo de Doblar y Sumar."""
    # Convertimos k a binario y eliminamos el prefijo '0b'
    k_bin = bin(k)[2:]
    
    # Acumulador Q inicializado en el punto en el infinito (0,0,0)
    Q = (0, 0, 0)
    
    # Usamos una variable temporal para el punto P
    P_temp = point
    
    # Leemos de derecha a izquierda (LSB a MSB)
    for bit in reversed(k_bin):
        if bit == '1':
            Q = jacobian_add(Q, P_temp)
        P_temp = jacobian_double(P_temp)
        
    return Q

# ==========================================
# UTILIDAD: JACOBIANA -> AFÍN
# ==========================================
def to_affine(jacobian_point):
    """Convierte el resultado Jacobiano (X, Y, Z) de vuelta a (x, y) 2D tradicional."""
    X, Y, Z = jacobian_point
    if Z == 0:
        return None # Punto en el infinito
    
    Z_inv = mod_inv(Z)
    Z_inv_sq = mod_mult(Z_inv, Z_inv)
    Z_inv_cb = mod_mult(Z_inv_sq, Z_inv)
    
    x = mod_mult(X, Z_inv_sq)
    y = mod_mult(Y, Z_inv_cb)
    
    return (x, y)

# ==========================================
# EJECUCIÓN PRINCIPAL (TEST)
# ==========================================
if __name__ == "__main__":
    # 1. Definimos nuestro punto Generador en formato Jacobiano (Z = 1)
    G_Jacobian = (G_X, G_Y, 1)
    
    # 2. Definimos una clave privada (escalar k) arbitrariamente grande
    # En la vida real, este número de 256 bits se genera aleatoriamente.
    private_key = 0x18E14A7B6A307F426A94F8114701E7C8E774E7F9A47E2C2035DB29A206321725
    
    print("Iniciando multiplicación escalar (k * G)...")
    print(f"k (Clave Privada): {hex(private_key)}\n")
    
    # 3. Calculamos la clave pública usando toda nuestra torre de ECC
    public_key_jacobian = double_and_add(private_key, G_Jacobian)
    
    # 4. Volvemos a coordenadas 2D comprensibles (x, y)
    public_key_affine = to_affine(public_key_jacobian)
    
    print("=== RESULTADO (Clave Pública) ===")
    print(f"X: {hex(public_key_affine[0])}")
    print(f"Y: {hex(public_key_affine[1])}")
