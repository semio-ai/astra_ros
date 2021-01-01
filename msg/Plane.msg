# From the Astra SDK documentation:
# Struct representing a geometric 3-dimensional plane. 
# The normal of the plane is defined to be (A, B, C). The plane coefficients and real
# world coordinates (x, y, and z) are related by the plane equation: 
#   Ax + By + Cz + D = 0 
# Remarks
#   The vector v = (A, B, C) represents the normal to the plane and D represents the
#   signed distance along the normal between the plane and the origin of the coordinate
#   system. The the origin of the coordinate system is the camera's depth sensor 

float32 a
float32 b
float32 c
float32 d