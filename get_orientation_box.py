import numpy as np

def orientacion_rectangulo(A, B, C, D):
    """
    Calculates the orientation (roll, pitch, yaw) of a rectangle
    defined by four 3D points: A, B, C, D.
    """

    # Convertir a arrays
    A, B, C, D = map(np.array, [A, B, C, D])

    # Vectores del plano
    u = B - A
    v = D - A

    # Vector normal (producto cruzado)
    n = np.cross(u, v)
    n = n / np.linalg.norm(n)

    # Calcular ángulos (en radianes)
    roll = np.arctan2(n[1], n[2])
    pitch = np.arctan2(-n[0], np.sqrt(n[1]**2 + n[2]**2))
    yaw = np.arctan2(u[1], u[0])

    # Convertir a grados
    roll, pitch, yaw = np.degrees([roll, pitch, yaw])

    return roll, pitch, yaw

A = (89.7, -282.4, 443)
B = (-173.8, -189.4, 453.6)
C = (-100.8, -48, 252.2)
D = (138.3, -123.2, 254.1)

roll, pitch, yaw = orientacion_rectangulo(A, B, C, D)
print(f"Orientation (degree): Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")