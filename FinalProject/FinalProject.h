#pragma once

namespace Graphics {

	class Vector3D;
	class Sphere;
	class Ray;
	class Quaternion;
	class Transforms;
	class Matrix3x3;

	enum class QuaternionType 
	{
		FROM_COMPONENTS,   
		FROM_AXIS_ANGLE    
	};

	typedef float Scalar;	

	void test();

	Vector3D operator-(const Vector3D& vector1, const Vector3D& vector2);
	Vector3D operator+(const Vector3D& vector1, const Vector3D& vector2);
	Vector3D operator*(const Scalar& k, const Vector3D& vector);
	Vector3D operator*(const Vector3D& vector, const Scalar& k);
	bool operator==(const Vector3D& vector1, const Vector3D& vector2);
	
	void printVector3D(const Vector3D& vector);
	void printQuaternion(const Quaternion& quaternion);
	void printQuaternionAxisAndAngle(const Quaternion& quaternion);
	void printMatrix3x3(const Matrix3x3& matrix);

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

		Scalar squaredNorm() const;
		Scalar norm() const;
		Vector3D normalize();

		//getters and setters
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

	class Matrix3x3 
	{
	private:
		Vector3D firstColumn;
		Vector3D secondColumn;
		Vector3D thirdColumn;
		
	public:
		Matrix3x3();
		Matrix3x3(Vector3D firstColumn, Vector3D econdColumn, Vector3D thirdColumn);

		Matrix3x3 operator*(const Matrix3x3& other) const;
		Vector3D operator*(const Vector3D& vector) const;

		static Matrix3x3 identity();

		Vector3D getFirstColumn() const { return firstColumn; }
		Vector3D getSecondColumn() const { return secondColumn; }
		Vector3D getThirdColumn() const { return thirdColumn; }
	};


	// Quaternion
	Quaternion operator*(const Scalar& k, const Quaternion& quat);
	Quaternion operator*(const Quaternion& quat, const Scalar& k);

	Quaternion operator+(const Quaternion& quaternion1, const Quaternion& quaternion2);
	Quaternion operator-(const Quaternion& quaternion1, const Quaternion& quaternion2);
	Quaternion operator*(const Quaternion& quaternion1, const Quaternion& quaternion2);

	bool operator==(const Quaternion& quaternion1, const Quaternion& quaternion2);

	Scalar dot(const Quaternion& quaternion1, const Quaternion& quaternion2);

	// value = 0.0f -> quaternion1, value = 1.0f -> quaternion2
	Quaternion nlerp(const Quaternion& quaternion1, const Quaternion& quaternion2, Scalar value);
	Quaternion slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, Scalar value);

	bool areEqual(const Quaternion& quaternion1, const Quaternion& quaternion2);

	
	/*
	In matematica, i quaternioni sono entit� introdotte da William Rowan Hamilton nel 1843 come estensioni dei numeri complessi. Ipercomplessi, spazio H.
	Un quaternione � un oggetto formale del tipo:
	Quaternion = ai + bj + ck + d, dove a,b,c,d sono numeri reali e i,j,k sono dei simboli che si comportano in modo simile all'unit� immaginaria dei numeri complessi.
	Re(Quaternion) = d
	Im(Quaternion) = ai + bj + ck
	
	Un quaterniore a norma 1 rappresenta una rotazione intorno all'asse definito dal vettore img moltiplicato per seno di alpha/2, dove alpha � l'angolo di rotazione. Il valore re rappresenta il coseno di alpha/2.
	q = sin(alpha/2) * axis + cos(alpha/2)
	Questa sarebbe la trasformazione da asse angolo a quaternione.
	*/
	class Quaternion {

	private:
		Vector3D img;
		Scalar re;
		bool isRotation();

	public:
		static const Quaternion IDENTITY;
		static const Quaternion ZERO;

		Quaternion();
		Quaternion(Scalar imgX, Scalar imgY, Scalar imgZ, Scalar re); 
		Quaternion(Vector3D vector, Scalar scalar, QuaternionType type);
		Quaternion(Scalar roll, Scalar pitch, Scalar yaw);
		Quaternion(Matrix3x3 rotationMatrix);

		Scalar squaredNorm() const;
		Scalar norm() const;
		Quaternion normalized();
		void normalize();
		void flip();

		Quaternion conjugated(); // return a new quaternion with the imaginary part negated
		void conjugate(); // set the imaginary part negated of this quaternion
		Quaternion inverse();

		Vector3D rotate(const Vector3D& vector);

		Vector3D getAxis() const;
		Scalar getAngle() const;
		Matrix3x3 exportToMatrix3x3() const;

		// Getters and Setters
		Scalar getRe() const { return re; }
		Vector3D getImg() const { return img; }
		void setRe(Scalar newRe) { re = newRe; }
		void setImg(Vector3D newImg) { img = newImg; }
	
	};

	class Transforms {

	};

}
