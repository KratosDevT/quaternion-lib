#pragma once

namespace Graphics {

	class Vector3D;
	class Sphere;
	class Ray;
	class Quaternion;
	class Transforms;

	typedef float Scalar;	

	void test();

	Vector3D operator-(const Vector3D& vector1, const Vector3D& vector2);
	Vector3D operator+(const Vector3D& vector1, const Vector3D& vector2);
	Vector3D operator*(const Scalar& k, const Vector3D& vector);
	Vector3D operator*(const Vector3D& vector, const Scalar& k);
	bool operator==(const Vector3D& vector1, const Vector3D& vector2);
	
	void printVector3D(const Vector3D& vector);

	Scalar dot(const Vector3D& vec1, const Vector3D& vec2);
	Vector3D cross(const Vector3D& vec1, const Vector3D& vec2);
	Scalar distance(const Vector3D& vec1, const Vector3D& vec2);
	Vector3D intersect(const Ray& ray, const Sphere& sphere);

	class Vector3D {

	private:
		Scalar x;
		Scalar y;
		Scalar z;

	public:
		static const Vector3D ORIGIN;
		static const Vector3D RIGHT;
		static const Vector3D LEFT;
		static const Vector3D UP;
		static const Vector3D DOWN;
		static const Vector3D FORWARD;
		static const Vector3D BACKWARD;
		static const Vector3D MISSED;

		Vector3D();
		Vector3D(Scalar x, Scalar y, Scalar z);

		Scalar squaredNorm();
		Scalar norm();
		Vector3D normalize();

		Scalar getX() const { return x; }
		Scalar getY() const { return y; }
		Scalar getZ() const { return z; }

		void setX(Scalar newX) { x = newX; }
		void setY(Scalar newY) { y = newY; }
		void setZ(Scalar newZ) { z = newZ; }
	};

	class Sphere {

	public:
		Vector3D center;
		Scalar radius;
	};

	class Ray {

	public:
		Ray();
		Ray(Vector3D vec1, Vector3D vec2);
		Vector3D start;
		Vector3D dir;
	};

	/*
	In matematica, i quaternioni sono entit� introdotte da William Rowan Hamilton nel 1843 come estensioni dei numeri complessi.
	
	Un quaternione � un oggetto formale del tipo:
	
	Quaternion = ai + bj + ck + d, dove a,b,c,d sono numeri reali e i,j,k sono dei simboli che si comportano in modo simile all'unit� immaginaria dei numeri complessi.
	
	Re(Quaternion) = d
	Im(Quaternion) = ai + bj + ck
	*/
	class Quaternion {

	private:
		Vector3D img;
		Scalar real;

	public:
		Quaternion();
		Quaternion(Scalar real, Vector3D img);

		Quaternion operator*(const Quaternion& other);
		Quaternion operator+(const Quaternion& other);
		Quaternion operator-(const Quaternion& other);
		
		Scalar getReal() const { return real; }
		Vector3D getImg() const { return img; }
		
		void setReal(Scalar newReal) { real = newReal; }
		void setImg(Vector3D newImg) { img = newImg; }

		Quaternion conjugate();
		Vector3D rotate(const Vector3D& vector);
	
	};

	class Transforms {

	};

}
