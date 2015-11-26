import math

def quat_mult( q1, q2 ):
  w1, x1, y1, z1 = q1
  w2, x2, y2, z2 = q2
  w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
  x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
  y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
  z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
  return w, x, y, z

def normalise(v, tolerance=0.00001):
  mag2 = sum(n * n for n in v)
  if abs(mag2 - 1.0) > tolerance:
    mag = math.sqrt(mag2)
    v = tuple(n / mag for n in v)
  return v

def cross_prod(u, v):
  return (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])

def dot_prod(u, v):
  return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]

def quat_conjugate(q):
  q = normalise(q)
  w, x, y, z = q
  return (w, -x, -y, -z)

def qv_mult(q1, v1):
  v1 = normalise(v1)
  q2 = (0.0,) + v1
  return quat_mult(quat_mult(q1, q2), quat_conjugate(q1))[1:]

def vect_to_quant( vect, ref ):
  conv = normalise(cross_prod(vect,ref))
  q = (-math.acos(dot_prod(vect,ref)),) + conv
  return normalise(q)

def set_orient( rot_x, rot_y, rot_z ):
  quat_x = (math.cos(math.radians(rot_x)/2.0), math.sin(math.radians(rot_x)/2.0), 0.0, 0.0)
  quat_y = (math.cos(math.radians(rot_y)/2.0), 0.0, math.sin(math.radians(rot_y)/2.0), 0.0)
  quat_z = (math.cos(math.radians(rot_z)/2.0), 0.0, 0.0, math.sin(math.radians(rot_z)/2.0))
  
  multp1 = quat_mult(quat_x, quat_y)
  return quat_mult(multp1, quat_z)

