#include "quat_class.h"

float Quat::sqrt_(float S)
{
	if (S < 0)
		return -1;
	if (S == 0 || S == 1)
		return S;
	float left = 0, right = S, mid, epsilon = 1e-6;
	while (right - left > epsilon)
	{
		mid = (left + right) / 2;
		if (mid * mid > S)
			right = mid;
		else
			left = mid;
	}
	return (left + right) / 2;
}

float Quat::sin_(float x)
{
	x = x - static_cast< int >(x / TWO_PI) * TWO_PI;

	if (x > PI)
		x -= TWO_PI;
	if (x < -PI)
		x += TWO_PI;

	float result = x;
	float term = x;
	float n = 1.0f;
	while (true)
	{
		term *= -x * x / ((2.0f * n) * (2.0f * n + 1.0f));
		result += term;
		if (term < 1e-6f && term > -1e-6f)
			break;
		n++;
	}
	return result;
}

float Quat::cos_(float x)
{
	x = x - static_cast< int >(x / TWO_PI) * TWO_PI;

	if (x > PI)
		x -= TWO_PI;
	if (x < -PI)
		x += TWO_PI;

	float result = 1.0f;
	float term = 1.0f;
	float n = 1.0f;
	while (true)
	{
		term *= -x * x / ((2.0f * n - 1.0f) * (2.0f * n));
		result += term;
		if (term < 1e-6f && term > -1e-6f)
			break;
		n++;
	}
	return result;
}


Quat::Quat(float a, vector3_t vec) : comp{ a, vec.x, vec.y, vec.z }, cp{ a, vec.x, vec.y, vec.z } {}
Quat::Quat(float a, float b, float c, float d) : comp{ a, b, c, d }, cp{ a, b, c, d } {}

Quat::Quat(float T, bool is_radian, vector3_t vec) : comp{}, cp{}
{
	rotation_constructor_call = 1;
	if (!is_radian)
		T /= RADIAN;
	float norm = sqrt_(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	vec.x /= norm;
	vec.y /= norm;
	vec.z /= norm;
	float cosT = cos_(T / 2.0f);
	float sinT = sin_(T / 2.0f);
	comp.a = cosT;
	comp.b = vec.x * sinT;
	comp.c = vec.y * sinT;
	comp.d = vec.z * sinT;
	cp.a = cosT;
	cp.vec.x = vec.x * sinT;
	cp.vec.y = vec.y * sinT;
	cp.vec.z = vec.z * sinT;
}

Quat::Quat() : comp{}, cp{} {}

float Quat::scalar_mult(const Quat& other) const
{
	return cp.vec.x * other.cp.vec.x + cp.vec.y * other.cp.vec.y + cp.vec.z * other.cp.vec.z;
}
float Quat::scalar_mult(const vector3_t& vec) const
{
	return cp.vec.x * vec.x + cp.vec.y * vec.y + cp.vec.z * vec.z;
}
Quat::vector3_t Quat::vec_by_scalar(float scalar, const Quat& other)
{
	return { other.cp.vec.x * scalar, other.cp.vec.y * scalar, other.cp.vec.z * scalar };
}

Quat::vector3_t Quat::vec_by_scalar(float scalar) const
{
	return { cp.vec.x * scalar, cp.vec.y * scalar, cp.vec.z * scalar };
}

Quat::vector3_t Quat::vec_mul(const Quat& other) const
{
	return { cp.vec.y * other.cp.vec.z - other.cp.vec.y * cp.vec.z,
			 cp.vec.z * other.cp.vec.x - other.cp.vec.z * cp.vec.x,
			 cp.vec.x * other.cp.vec.y - other.cp.vec.x * cp.vec.y };
}
Quat::vector3_t Quat::vec_mul(const vector3_t& vec) const
{
	return { cp.vec.y * vec.z - vec.y * cp.vec.z,
			 cp.vec.z * vec.x - vec.z * cp.vec.x,
			 cp.vec.x * vec.y - vec.x * cp.vec.y };
}


Quat::operator float() const
{
	return sqrt_(comp.a * comp.a + comp.b * comp.b + comp.c * comp.c + comp.d * comp.d);
}

Quat& Quat::operator+=(const Quat& other)
{
	comp.a += other.comp.a;
	comp.b += other.comp.b;
	comp.c += other.comp.c;
	comp.d += other.comp.d;
	cp.a += other.cp.a;
	cp.vec.x += other.cp.vec.x;
	cp.vec.y += other.cp.vec.y;
	cp.vec.z += other.cp.vec.z;
	return *this;
}
Quat Quat::operator+(const Quat& other) const
{
	return Quat(comp.a + other.comp.a, comp.b + other.comp.b, comp.c + other.comp.c, comp.d + other.comp.d);
}

Quat& Quat::operator-=(const Quat& other)
{
	comp.a -= other.comp.a;
	comp.b -= other.comp.b;
	comp.c -= other.comp.c;
	comp.d -= other.comp.d;
	cp.a -= other.cp.a;
	cp.vec.x -= other.cp.vec.x;
	cp.vec.y -= other.cp.vec.y;
	cp.vec.z -= other.cp.vec.z;
	return *this;
}
Quat Quat::operator-(const Quat& other) const
{
	return Quat(comp.a - other.comp.a, comp.b - other.comp.b, comp.c - other.comp.c, comp.d - other.comp.d);
}

bool Quat::operator==(const Quat& other) const
{
	return comp.a == other.comp.a && comp.b == other.comp.b && comp.c == other.comp.c && comp.d == other.comp.d;
}

bool Quat::operator!=(const Quat& other) const
{
	return !(*this == other);
}

Quat Quat::operator~()
{
	comp.b *= -1.0f;
	comp.c *= -1.0f;
	comp.d *= -1.0f;
	cp.vec.x *= -1.0f;
	cp.vec.y *= -1.0f;
	cp.vec.z *= -1.0f;
	return *this;
}
Quat Quat::operator*(const Quat& other) const
{
	float sc_mult = scalar_mult(other);
	float new_sc = cp.a * other.cp.a - sc_mult;
	vector3_t v1 = vec_by_scalar(cp.a, other);
	vector3_t v2 = vec_by_scalar(other.cp.a);
	vector3_t v3 = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	vector3_t v4 = vec_mul(other);
	return Quat(new_sc, { v3.x + v4.x, v3.y + v4.y, v3.z + v4.z });
}
Quat& Quat::operator*=(const Quat& other)
{
	float sc_mult = scalar_mult(other);
	float new_sc = cp.a * other.cp.a - sc_mult;
	vector3_t v1 = vec_by_scalar(cp.a, other);
	vector3_t v2 = vec_by_scalar(other.cp.a);
	vector3_t v3 = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	vector3_t v4 = vec_mul(other);
	comp.a = new_sc;
	comp.b = v3.x + v4.x;
	comp.c = v3.y + v4.y;
	comp.d = v3.z + v4.z;
	cp.a = new_sc;
	cp.vec.x = v3.x + v4.x;
	cp.vec.y = v3.y + v4.y;
	cp.vec.z = v3.z + v4.z;
	return *this;
}
Quat& Quat::operator*=(const vector3_t& vec)
{
	float sc_mult = scalar_mult(vec);
	float sc = cp.a;
	float new_sc =-sc_mult;
	vector3_t v1 = {vec.x*sc,vec.y*sc,vec.z*sc };
	vector3_t v2 = vec_by_scalar(0.0f);
	vector3_t v3 = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	vector3_t v4 = vec_mul(vec);
	comp.a = new_sc;
	comp.b = v3.x + v4.x;
	comp.c = v3.y + v4.y;
	comp.d = v3.z + v4.z;
	cp.a = new_sc;
	cp.vec.x = v3.x + v4.x;
	cp.vec.y = v3.y + v4.y;
	cp.vec.z = v3.z + v4.z;
	return *this;
}
Quat& Quat::operator*=(float scalar)
{
	comp.a *= scalar, cp.a *= scalar;
	comp.b *= scalar, cp.vec.x *= scalar;
	comp.c *= scalar, cp.vec.y *= scalar;
	comp.d *= scalar, cp.vec.z *= scalar;
	return *this;
}
float* Quat::data() const
{
	float*pointer_arr = new float[4];
	pointer_arr[0] = comp.b;
	pointer_arr[1] = comp.c;
	pointer_arr[2] = comp.d;
	pointer_arr[3] = comp.a;
	return pointer_arr;
}
float** Quat::rotation_matrix() const
{
	float x,y,z,w;
	float norm = sqrt_(cp.a*cp.a+cp.vec.x*cp.vec.x+cp.vec.y*cp.vec.y+cp.vec.z*cp.vec.z);
	if (cp.vec.x == 0 && cp.vec.y == 0 && cp.vec.z == 0) return nullptr;
	if (norm != 1.0f && !rotation_constructor_call)
	{
		x = X/norm;
		y = Y/norm;
		z = Z/norm;
		w = W/norm;
	}
	else
	{
		x = X;
		y = Y;
		z = Z;
		w = W;
	}
	float** matrix = new float*[4];
	matrix[0] = new float[4]{ 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w, 0.0f };
	matrix[1] = new float[4]{ 2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w, 0.0f };
	matrix[2] = new float[4]{ 2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y, 0.0f };
	matrix[3] = new float[4]{ 0.0f, 0.0f, 0.0f, 1.0f };
	return matrix;

}
float** Quat::matrix() const
{
	float** matrix = new float*[4];
	matrix[0] = new float[4]{ comp.a, -comp.b, -comp.c, -comp.d };
	matrix[1] = new float[4]{ comp.b, comp.a, -comp.d, comp.c };
	matrix[2] = new float[4]{ comp.c, comp.d, comp.a, -comp.b };
	matrix[3] = new float[4]{ comp.d, -comp.c, comp.b, comp.a };
	return matrix;
}

void Quat::free_matrix(float** matrix)
{
	for (int i = 0; i < 4; ++i)
	{
		delete[] matrix[i];
	}
	delete[] matrix;
}
void Quat::free_data(float* arr)
{
	delete arr;
}