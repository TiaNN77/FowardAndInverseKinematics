import sympy as sp
from sympy import cos, sin, simplify, pprint

def compute_transformation_matrices():
    """
    Computes T_123 = A1 × A2 × A3, T_45 = A4 × A5, and T_full = A1 × A2 × A3 × A4 × A5
    """
    # Define symbolic variables
    q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
    L1, L2, L3, L4 = sp.symbols('L1 L2 L3 L4')
    
    # Define individual transformation matrices
    A1 = sp.Matrix([
        [cos(q1), -sin(q1), 0, 0],
        [sin(q1),  cos(q1), 0, 0],
        [0,        0,       1, L1],
        [0,        0,       0, 1]
    ])
    
    A2 = sp.Matrix([
        [cos(q2), -sin(q2), 0, 0],
        [0,        0,      -1, 0],
        [sin(q2),  cos(q2), 0, 0],
        [0,        0,       0, 1]
    ])
    
    A3 = sp.Matrix([
        [cos(q3), -sin(q3), 0, L2],
        [sin(q3),  cos(q3), 0, 0],
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ])
    
    A4 = sp.Matrix([
        [cos(q4), -sin(q4), 0, L3],
        [sin(q4),  cos(q4), 0, 0],
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ])
    
    A5 = sp.Matrix([
        [cos(q5), -sin(q5), 0, 0],
        [0,        0,      1, L4],
        [-sin(q5),  -cos(q5), 0, 0],
        [0,        0,       0, 1]
    ])
    
    # Compute transformation matrices
    print("T_123 = A1 × A2 × A3:")
    T_123 = simplify(A1 * A2 * A3)
    pprint(T_123)
    
    print("\nT_45 = A4 × A5:")
    T_45 = simplify(A4 * A5)
    pprint(T_45)
    
    print("\nT_full = A1 × A2 × A3 × A4 × A5:")
    T_full = simplify(A1 * A2 * A3 * A4 * A5)
    pprint(T_full)
    
    return T_123, T_45, T_full

if __name__ == "__main__":
    T_123, T_45, T_full = compute_transformation_matrices()