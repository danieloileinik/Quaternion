#pragma once
#define PI 3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f
#define RADIAN 57.2958f
#define X cp.vec.x
#define Y cp.vec.y
#define Z cp.vec.z
#define W cp.a

class Quat
{
    struct components
    {
        float a;
        float b;
        float c;
        float d;
    };
    components comp;

public:
    struct vector3_t
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

private:
    struct comp_pair
    {
        float a;
        vector3_t vec;
    };
    comp_pair cp;

    static float sqrt_(float S);
    static float sin_(float x);
    static float cos_(float x);
    short rotation_constructor_call = 0;
public:
    Quat(float a, vector3_t vec);
    Quat(float a, float b, float c, float d);
    Quat(float T, bool is_radian, vector3_t vec);
    Quat();
    float scalar_mult(const Quat& other) const;
    float scalar_mult(const vector3_t& vec) const;
    static vector3_t vec_by_scalar(float scalar, const Quat& other);
    vector3_t vec_by_scalar(float scalar) const;
    vector3_t vec_mul(const Quat& other) const;
    vector3_t vec_mul(const vector3_t& vec) const;
    explicit operator float() const;
    Quat operator+(const Quat& other) const;
    Quat& operator+=(const Quat& other);
    Quat operator-(const Quat& other) const;
    Quat& operator-=(const Quat& other);
    Quat operator*(const Quat& other) const;
    Quat& operator*=(const Quat& other);
    Quat& operator*=(float scalar);
    Quat& operator*=(const vector3_t& vec) ;
    bool operator==(const Quat& other) const;
    bool operator!=(const Quat& other) const;
    Quat operator~();
    float* data() const;
    float** rotation_matrix() const;
    float** matrix() const;
    static void free_matrix(float** matrix);
    static void free_data(float* arr);

};
