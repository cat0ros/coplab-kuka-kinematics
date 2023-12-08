import numpy as np
import cmath

def is_on_line(point1, point2, point3):
    vector_a = point2 - point1
    vector_b = point3 - point1
    vec_cross = np.cross(vector_a, vector_b)
    return np.all((vec_cross == 0))
    
def build(point1, point2, point3):
    if not is_on_line(point1, point2, point3):
        chast = point1[1] * (point2[2] - point3[2]) + point2[1] * (point3[2] - point1[2]) + point3[1] * (point1[2] - point2[2])
        y_m_0 = point1[2] * (point2[1]**2 + point2[2]**2 - point3[1]**2 - point3[2]**2)
        y_m_1 = point2[2] * (point3[1]**2 + point3[2]**2 - point1[1]**2 - point1[2]**2)
        y_m_2 = point3[2] * (point1[1]**2 + point1[2]**2 - point2[1]**2 - point2[2]**2)
        z_m_0 = point1[1] * (point2[1]**2 + point2[2]**2 - point3[1]**2 - point3[2]**2)
        z_m_1 = point2[1] * (point3[1]**2 + point3[2]**2 - point1[1]**2 - point1[2]**2)
        z_m_2 = point3[1] * (point1[1]**2 + point1[2]**2 - point2[1]**2 - point2[2]**2)
        y_m = (-1/2) * (y_m_0 + y_m_1 + y_m_2) / (chast)
        z_m = (1/2) * (z_m_0 + z_m_1 + z_m_2) / (chast)
        y_m_point1 = (y_m - point1[1])**2
        z_m_point1 = (z_m - point1[2])**2
        radius = np.sqrt(y_m_point1 + z_m_point1)
        return (point1[0], y_m, z_m, radius)
    else:
        print('Cant paint circle because three points is on one line.')
    return (None, None, None)

def calculate_points(count, radius, center):
    x, y, z = center
    coordinates = []
    for i in range(count):
        r = cmath.rect(radius, (2*cmath.pi)*(i/count))
        coordinates.append([x, round(y+r.imag, 2), round(z+r.real, 2)])
    
    return coordinates