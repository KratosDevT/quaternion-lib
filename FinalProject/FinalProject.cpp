// FinalProject.cpp : Defines the functions for the static library.
//

#include "pch.h"
#include "framework.h"
#include "FinalProject.h"
#include <iostream>

namespace Graphics
{
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
		return (std::abs(vector1.getX() - vector2.getX()) < FLT_EPSILON) && ((vector1.getY() - vector2.getY()) < FLT_EPSILON) && ((vector1.getZ() - vector2.getZ()) < FLT_EPSILON);
	}

	

	//print
	void printVector3D(const Vector3D& vector)
	{
		std::cout << "[" << vector.getX() << "," << vector.getY() << "," << vector.getZ() << "]" << std::endl;
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

	Scalar Vector3D::squaredNorm()
	{
		return dot(*this, *this);
	}

	Scalar Vector3D::norm()
	{
		return std::sqrt(Vector3D::squaredNorm());
	}

	Vector3D Vector3D::normalize()
	{
		return *this * (1/norm());
	}

	Ray::Ray(){}
	Ray::Ray(Vector3D vec1, Vector3D vec2) : start{ vec1 }, dir{ vec2 }	{}

	const Quaternion Quaternion::IDENTITY{ 0, 0, 0, 1 };
	const Quaternion Quaternion::ZERO{ 0, 0, 0, 0 };

	void printQuaternion(const Quaternion& quaternion)
	{
		std::cout << "[" << quaternion.getImg().getX() << ", " << quaternion.getImg().getY() << ", " << quaternion.getImg().getZ() << ", " << quaternion.getRe() << "] -> ";
		std::cout << "[" << quaternion.getImg().getX() << "i+" << quaternion.getImg().getY() << "j+" << quaternion.getImg().getZ() << "k+" << quaternion.getRe() << "]" << std::endl;
	}

	Quaternion::Quaternion() : img{ Vector3D::ORIGIN }, re{ 0 } {}
	Quaternion::Quaternion(Scalar _x, Scalar _y, Scalar _z, Scalar _re) : img{ Vector3D(_x, _y, _z) }, re{ _re } {}

    Quaternion::Quaternion(Vector3D vector, Scalar scalar, QuaternionType type) : re(0)
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
				img = vector.normalize() * std::sin(scalar / 2.0f);
				re = std::cos(scalar / 2.0f);
				break;

			default:
				std::cerr << "Invalid QuaternionType provided." << std::endl;
		}
    }

	bool Quaternion::operator==(const Quaternion& other)
	{
		return (this->getRe() == other.getRe()) && (this->getImg()==other.getImg());
	}

	Quaternion Quaternion::operator+(const Quaternion& other) 
	{
		return Quaternion(other.getImg().getX()+img.getX(), other.getImg().getY()+img.getY(), other.getImg().getZ()+img.getZ(), other.getRe()+re);
	}
	
	Quaternion Quaternion::operator*(const Quaternion& other)
	{
		Scalar _re = (this->getRe() * other.getRe()) - dot(this->getImg(), other.getImg());
		Vector3D _img = (this->getRe() * other.getImg()) + (other.getRe() * this->getImg()) + cross(this->getImg(), other.getImg());
		return Quaternion(_img, _re, QuaternionType::FROM_COMPONENTS);
	}

	Quaternion operator*(const Scalar& value, const Quaternion& quat)
	{
		return Quaternion(quat.getImg() * value, quat.getRe() * value, QuaternionType::FROM_COMPONENTS);
	}

	Quaternion operator*(const Quaternion& quat, const Scalar& value)
	{
		return value * quat;
	}

	Quaternion Quaternion::operator-(const Quaternion& other)
	{
		return Quaternion(img.getX() - other.getImg().getX() , img.getY() - other.getImg().getY() , img.getZ() - other.getImg().getZ() , re - other.getRe());
	}

	Quaternion Quaternion::conjugate()
	{
		return Quaternion(-img.getX(), -img.getY(), -img.getZ(), re);
	}

	Scalar Quaternion::norm() 
	{
		return std::sqrt(this->squaredNorm());
	}

	Scalar Quaternion::squaredNorm() 
	{
		return (this->getRe() * this->getRe()) + this->getImg().squaredNorm();
	}

	Quaternion Quaternion::normalize() 
	{
		Scalar norm = this->norm();
		if (norm - 0.0f < FLT_EPSILON)
		{
			return Quaternion::ZERO;
		}

		return Quaternion(this->getImg() * (1/norm), this->getRe() / norm, QuaternionType::FROM_COMPONENTS);
	}

	bool Quaternion::isRotation()
	{
		if ((this->squaredNorm() - 1.0f) < FLT_EPSILON)
		{
			return true;
		}
		else
		{
			return false;
		}

	}

	float cleanFloat(float value) {
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
			Quaternion q_conjugate = this->conjugate();
			//std::cout << "q_conjugate: ";
			//printQuaternion(q_conjugate);
			Quaternion result = cleanQuaternion((*this * q_vector) * q_conjugate);
			//std::cout << "result: ";
			//printQuaternion(result);
			return result.getImg();
		}
		else
		{
			std::cerr << "Quaternion is not a valid rotation quaternion." << std::endl;
		}
	}

	Quaternion Quaternion::inverse() 
	{
		return this->conjugate() * (1 / this->squaredNorm());
	}


}


