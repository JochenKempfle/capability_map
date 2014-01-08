#ifndef VECTOR_H
#define VECTOR_H

namespace capability_map_generator
{

// small helper class used for coordinates and vectors
class Vector
{
  public:
    Vector() { }
    ~Vector() { }
    Vector(double xVal, double yVal, double zVal) : x(xVal), y(yVal), z(zVal) { }
    Vector(const Vector &rhs) : x(rhs.x), y(rhs.y), z(rhs.z) { }
    
    inline Vector operator*(double rhs) const { return Vector(x * rhs, y * rhs, z * rhs); }
    inline Vector operator+(const Vector &rhs) const { return Vector(x + rhs.x, y + rhs.y, z + rhs.z); }
    
    inline Vector cross(const Vector &rhs) const 
    {
        return Vector(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
    }
    
    inline double dot(const Vector &rhs) const 
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    
    double x, y, z;
};

}

#endif // VECTOR_H
