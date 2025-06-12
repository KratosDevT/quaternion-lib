// FinalProject.cpp : Defines the functions for the static library.
//

#include "pch.h"
#include "framework.h"
#include "FinalProject.h"
#include <iostream>
#include <cassert>

namespace Graphics
{
	const double PI = 4.0 * atan(1.0);
	float cleanFloat(float value);
	const Vector3D Vector3D::ORIGIN{ 0, 0, 0 };

	/// <summary>Zero vector representing origin point (0, 0, 0)</summary>
	const Vector3D Vector3D::RIGHT{ 1, 0, 0 };
	const Vector3D Vector3D::LEFT{ -1, 0, 0 };
	const Vector3D Vector3D::UP{ 0, 1, 0 };
	const Vector3D Vector3D::DOWN{ 0, -1, 0 };
	const Vector3D Vector3D::FORWARD{ 0, 0, 1 };
	const Vector3D Vector3D::BACKWARD{ 0, 0, -1 };
	const Vector3D Vector3D::MISSED{ NAN, NAN, NAN };

	void test() 
	{
		std::cout << "test OK" << std::endl;
	}

	//ctors
	Vector3D::Vector3D() : x(0), y(0), z(0) {}
	Vector3D::Vector3D(Scalar _x, Scalar _y, Scalar _z) : x(_x), y(_y), z(_z) {}

	//ops
	Vector3D operator-(const Vector3D& vec1, const Vector3D& vec2)
	{
		return Vector3D(vec1.getX() - vec2.getX(), vec1.getY() - vec2.getY(), vec1.getZ() - vec2.getZ());
	}

	Vector3D operator+(const Vector3D& vec1, const Vector3D& vec2)
	{
		return Vector3D(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY(), vec1.getZ() + vec2.getZ());
	}

	Vector3D operator*(const Scalar& k, const Vector3D& vec1)
	{
		return Vector3D(k * vec1.getX(), k * vec1.getY(), k * vec1.getZ());
	}

	Vector3D operator*(const Vector3D& vector, const Scalar& k)
	{
		return k * vector;
	}

	bool operator==(const Vector3D& vector1, const Vector3D& vector2)
	{
		return (std::abs(vector1.getX() - vector2.getX()) < FLT_EPSILON) && (std::abs(vector1.getY() - vector2.getY()) < FLT_EPSILON) && (std::abs(vector1.getZ() - vector2.getZ()) < FLT_EPSILON);
	}

	//print
	void printVector3D(const Vector3D& vector)
	{
		std::cout << "[" << vector.getX() << "," << vector.getY() << "," << vector.getZ() << "]";
	}

	Scalar dot(const Vector3D& vec1, const Vector3D& vec2)
	{
		return vec1.getX() * vec2.getX() + vec1.getY() * vec2.getY() + vec1.getZ() * vec2.getZ();
	}

	Vector3D cross(const Vector3D& vec1, const Vector3D& vec2)
	{
		return Vector3D(vec1.getY() * vec2.getZ() - vec1.getZ() * vec2.getY(),
			(-1) * (vec1.getX() * vec2.getZ() - vec1.getZ() * vec2.getX()),
			vec1.getX() * vec2.getY() - vec1.getY() * vec2.getX());
	}

	Scalar distance(const Vector3D& vec1, const Vector3D& vec2)
	{
		return Scalar();
	}

	Vector3D intersect(const Ray& ray, const Sphere& sphere)
	{
		Vector3D G = ray.start;
		Vector3D d = ray.dir;

		Vector3D C = sphere.center;
		Scalar r = sphere.radius;

		Scalar a = d.squaredNorm();
		Scalar b = dot(G - C, d) * 2;
		Scalar c = (G - C).squaredNorm() - r * r;

		Scalar delta = b * b - 4 * a * c;
		if (delta < 0) return Vector3D::MISSED;

		Scalar k = (-b - sqrt(delta)) / (2 * a);
		if (k < 0) return Vector3D::MISSED;

		return G + (k * d);
	}

	Scalar Vector3D::squaredNorm() const
	{
		return dot(*this, *this);
	}

	Scalar Vector3D::norm() const
	{
		return std::sqrt(Vector3D::squaredNorm());
	}

	Vector3D Vector3D::normalize()
	{
		Scalar n = norm();
		if (std::abs(n - 0.0f) < FLT_EPSILON)
		{
			return Vector3D::ORIGIN; // Return zero vector if norm is zero
		}
		return *this * (1/norm());
	}

	Ray::Ray(){}
	Ray::Ray(Vector3D vec1, Vector3D vec2) : start{ vec1 }, dir{ vec2 }	{}

	Matrix3x3::Matrix3x3(Vector3D _firstColumn, Vector3D _secondColumn, Vector3D _thirdColumn) : firstColumn{ _firstColumn }, secondColumn{ _secondColumn }, thirdColumn{ _thirdColumn } {}

	Vector3D Matrix3x3::operator*(const Vector3D& vector) const
	{
		return Vector3D(
			firstColumn.getX() * vector.getX() + secondColumn.getX() * vector.getY() + thirdColumn.getX() * vector.getZ(),
			firstColumn.getY() * vector.getX() + secondColumn.getY() * vector.getY() + thirdColumn.getY() * vector.getZ(),
			firstColumn.getZ() * vector.getX() + secondColumn.getZ() * vector.getY() + thirdColumn.getZ() * vector.getZ()
		);
	}

	const Quaternion Quaternion::IDENTITY{ 0, 0, 0, 1 };
	const Quaternion Quaternion::ZERO{ 0, 0, 0, 0 };

	void printQuaternion(const Quaternion& quaternion)
	{
		std::cout << "[" << quaternion.getImg().getX() << ", " << quaternion.getImg().getY() << ", " << quaternion.getImg().getZ() << ", " << quaternion.getRe() << "] -> ";
		std::cout << "[" << quaternion.getImg().getX() << "i+" << quaternion.getImg().getY() << "j+" << quaternion.getImg().getZ() << "k+" << quaternion.getRe() << "]" << std::endl;
	}

	void printQuaternionAxisAndAngle(const Quaternion& quaternion)
	{
		Vector3D axis = quaternion.getAxis();
		Scalar angle = quaternion.getAngle();

		std::cout << "[ Axis: ";
		printVector3D(axis);
		std::cout << ", Angle: " << angle << " ]";
		std::cout << std::endl;
	}
	void printMatrix3x3(const Matrix3x3& matrix) {
		std::cout << "Matrix3x3: " << std::endl;
		std::cout << "[" << matrix.getFirstColumn().getX() << ", " << matrix.getSecondColumn().getX() << ", " << matrix.getThirdColumn().getX() << "]" << std::endl;
		std::cout << "[" << matrix.getFirstColumn().getY() << ", " << matrix.getSecondColumn().getY() << ", " << matrix.getThirdColumn().getY() << "]" << std::endl;
		std::cout << "[" << matrix.getFirstColumn().getZ() << ", " << matrix.getSecondColumn().getZ() << ", " << matrix.getThirdColumn().getZ() << "]" << std::endl;
	}

	Quaternion::Quaternion() : img{ Vector3D::ORIGIN }, re{ 0 } {}
	Quaternion::Quaternion(Scalar _x, Scalar _y, Scalar _z, Scalar _re) : img{ Vector3D(_x, _y, _z) }, re{ _re } {}

    Quaternion::Quaternion(Vector3D vector, Scalar scalar, QuaternionType type)
    {
		switch(type) 
		{
			case QuaternionType::FROM_COMPONENTS:
				//std::cout << "QuaternionType componenti" << std::endl;
				img = vector; 
				re = scalar;
				break;

			case QuaternionType::FROM_AXIS_ANGLE:
				//std::cout << "QuaternionType Asse angolo" << std::endl;
				img = vector.normalize() * cleanFloat(std::sin(scalar / 2.0f));
				re = cleanFloat(std::cos(scalar / 2.0f));
				break;

			default:
				re = 0;
				img = Vector3D::ORIGIN;
				std::cerr << "Invalid QuaternionType provided." << std::endl;
		}
    }

	Quaternion::Quaternion(Scalar roll, Scalar pitch, Scalar yaw) // convezione XYZ (roll, pitch, yaw)
	{
		// roll is rotation around x-axis, pitch around y-axis, yaw around z-axis
		// Convert Euler angles (roll, pitch, yaw) to quaternion

		re = std::cos(roll / 2.0f) * std::cos(pitch / 2.0f) * std::cos(yaw / 2.0f) - std::sin(roll / 2.0f) * std::sin(pitch / 2.0f) * std::sin(yaw / 2.0f);
		Scalar x = std::sin(roll / 2.0f) * std::cos(pitch / 2.0f) * std::cos(yaw / 2.0f) + std::cos(roll / 2.0f) * std::sin(pitch / 2.0f) * std::sin(yaw / 2.0f);
		Scalar y = std::cos(roll / 2.0f) * std::sin(pitch / 2.0f) * std::cos(yaw / 2.0f) - std::sin(roll / 2.0f) * std::cos(pitch / 2.0f) * std::sin(yaw / 2.0f);
		Scalar z = std::cos(roll / 2.0f) * std::cos(pitch / 2.0f) * std::sin(yaw / 2.0f) + std::sin(roll / 2.0f) * std::sin(pitch / 2.0f) * std::cos(yaw / 2.0f);
		img = Vector3D(x, y, z);
	}

	Quaternion operator*(const Scalar& value, const Quaternion& quat)
	{
		return Quaternion(quat.getImg() * value, quat.getRe() * value, QuaternionType::FROM_COMPONENTS);
	}

	Quaternion operator*(const Quaternion& quat, const Scalar& value)
	{
		return value * quat;
	}

	Quaternion operator+(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		return Quaternion(
			quaternion1.getImg().getX() + quaternion2.getImg().getX(), 
			quaternion1.getImg().getY() + quaternion2.getImg().getY(), 
			quaternion1.getImg().getZ() + quaternion2.getImg().getZ(),
			quaternion1.getRe() + quaternion2.getRe()
		);
	}

	Quaternion operator-(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		return Quaternion(
			quaternion1.getImg().getX() - quaternion2.getImg().getX(),
			quaternion1.getImg().getY() - quaternion2.getImg().getY(),
			quaternion1.getImg().getZ() - quaternion2.getImg().getZ(),
			quaternion1.getRe() - quaternion2.getRe()
		);
	}

	Quaternion operator*(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		Scalar _re = dot(quaternion1, quaternion2);
		Vector3D _img = (quaternion1.getRe() * quaternion2.getImg()) + (quaternion2.getRe() * quaternion1.getImg()) + cross(quaternion1.getImg(), quaternion2.getImg());
		return Quaternion(_img, _re, QuaternionType::FROM_COMPONENTS);
	}

	bool operator==(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		return (quaternion1.getRe() == quaternion2.getRe()) && (quaternion1.getImg() == quaternion2.getImg());
	}

	Scalar dot(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		return (quaternion1.getRe() * quaternion2.getRe()) - dot(quaternion1.getImg(), quaternion2.getImg());
	}

	//The normalized linear interpolation of two quaternions.
	Quaternion nlerp(const Quaternion& quaternion1, const Quaternion& quaternion2, Scalar interpolationValue)
	{
		assert(interpolationValue >= 0 || interpolationValue <= 1);
		Quaternion quaternion3 = quaternion2;
		if (dot(quaternion1, quaternion2) < 0) 
		{
			quaternion3.flip();
		}
		Quaternion result = ((1 - interpolationValue) * quaternion1) + (interpolationValue * quaternion3);
		/*std::cout << "nlerp result: ";
		printQuaternion(result);*/
		result.normalize();
		return result;
	}

	Matrix3x3 Quaternion::exportToMatrix3x3() const
	{
		Scalar imgX = this->getImg().getX();
		Scalar imgY = this->getImg().getY();
		Scalar imgZ = this->getImg().getZ();
		Scalar re = this->getRe();

		return Matrix3x3(
			Vector3D(1 - 2 * (imgY * imgY + imgZ * imgZ), 2 * (imgX * imgY + re * imgZ), 2 * (imgX * imgZ - re * imgY)),
			Vector3D(2 * (imgX * imgY - re * imgZ), 1 - 2 * (imgX * imgX + imgZ * imgZ), 2 * (imgY * imgZ + re * imgX)),
			Vector3D(2 * (imgX * imgZ + re * imgY), 2 * (imgY * imgZ + re * imgX), 1 - 2 * (imgX * imgX + imgY * imgY))
		);

	}

	//The spherical linear interpolation of two quaternions.
	Quaternion slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, Scalar interpolationValue)
	{
		assert(interpolationValue >= 0 || interpolationValue <= 1);
		Quaternion quaternion3 = quaternion2;
		if (dot(quaternion1, quaternion2) < 0)
		{
			quaternion3.flip();
		}
		Scalar theta = std::acos(dot(quaternion1, quaternion2));
		Quaternion result = (std::sin((1 - interpolationValue) * theta) / sin(theta)) * quaternion1 + (sin(interpolationValue * theta) / sin(theta)) * quaternion3;
		return result;
	}

	bool areEqual(const Quaternion& quaternion1, const Quaternion& quaternion2)
	{
		return (quaternion1 == quaternion2);
	}

	Quaternion Quaternion::conjugated()
	{
		return Quaternion(-img.getX(), -img.getY(), -img.getZ(), re);
	}

	void Quaternion::conjugate()
	{
		this->setImg(-1 * this->getImg());
	}

	Scalar Quaternion::norm() const
	{
		return std::sqrt(this->squaredNorm());
	}

	Scalar Quaternion::squaredNorm() const
	{
		return (this->getRe() * this->getRe()) + this->getImg().squaredNorm();
	}

	Quaternion Quaternion::normalized() 
	{
		Scalar norm = this->norm();
		if (std::abs(norm - 0.0f) < FLT_EPSILON)
		{
			return Quaternion::ZERO;
		}
		return Quaternion(this->getImg() * (1/norm), this->getRe() / norm, QuaternionType::FROM_COMPONENTS);
	}

	void Quaternion::normalize()
	{
		Scalar norm = this->norm();
		if (std::abs(norm - 0.0f) < FLT_EPSILON)
		{
			this->Quaternion::ZERO;
		}

		this->setImg(this->getImg() * (1.0f / norm));
		this->setRe(this->getRe() * (1.0f / norm));
	}

	void Quaternion::flip()
	{
		this->setImg(-1 * this->getImg());
		this->setRe(-1 * this->getRe());
	}

	bool Quaternion::isRotation()
	{
		if (this->squaredNorm() - 1.0f < FLT_EPSILON)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	float cleanFloat(float value) 
	{
		if (std::abs(value) < FLT_EPSILON) {
			return 0.0f;
		}
		return value;
	}

	Quaternion cleanQuaternion(const Quaternion& q) {
		return Quaternion(
			cleanFloat(q.getImg().getX()),
			cleanFloat(q.getImg().getY()),
			cleanFloat(q.getImg().getZ()),
			cleanFloat(q.getRe()));
	}

	Vector3D Quaternion::rotate(const Vector3D& vector)
	{
		if (this->isRotation())
		{
			//std::cout << "this: ";
			//printQuaternion(*this);
			Quaternion q_vector(vector.getX(), vector.getY(), vector.getZ(), 0);
			//std::cout << "q_vector: ";
			//printQuaternion(q_vector);
			Quaternion q_this = *this;
			q_this.normalize();
			Quaternion q_conjugate = q_this.conjugated();
			//std::cout << "q_conjugate: ";
			//printQuaternion(q_conjugate);
			Quaternion result = cleanQuaternion(q_this * q_vector * q_conjugate);
			//std::cout << "result: ";
			//printQuaternion(result);
			assert(std::abs(result.getRe() - 0.0f) < FLT_EPSILON);
			assert(result.getImg().squaredNorm() >= 0.0f);
			assert(result.getImg().squaredNorm() - vector.squaredNorm() <= 0.000001f);
			return result.getImg();
		}
		else
		{
			std::cerr << "Quaternion is not a valid rotation quaternion, squaredNorm: " << this->squaredNorm() << std::endl;
		}
	}

	Vector3D Quaternion::getAxis() const
	{
		Vector3D axis = this->getImg();
		return axis * (1 / axis.norm());
	}

	Scalar Quaternion::getAngle() const
	{
		Scalar norm = this->norm();
		Scalar realPart = this->getRe();
		Scalar realPartNormalized = realPart * (1 / norm);
		return 2.0f * std::acos(realPartNormalized) * (180 / PI);
	}

	Quaternion Quaternion::inverse() 
	{
		return this->conjugated() * (1 / this->squaredNorm());
	}
}


