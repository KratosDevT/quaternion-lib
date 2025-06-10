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
		return (vector1.getX() == vector2.getX()) && (vector1.getY() == vector2.getY()) && (vector1.getZ() == vector2.getZ());
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
		Vector3D result;
		result.setX(getX() / norm());
		result.setY(getY() / norm());
		result.setZ(getZ() / norm());
		return result;
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
	Quaternion::Quaternion(Vector3D _img, Scalar _re) : img{ _img }, re{ _re } {}
	Quaternion::Quaternion(Scalar _x, Scalar _y, Scalar _z, Scalar _re) : img{ Vector3D(_x, _y, _z) }, re{ _re } {}

	bool Quaternion::operator==(const Quaternion& other)
	{
		return (this->getRe() == other.getRe()) && (this->getImg()==other.getImg());
	}

	Quaternion Quaternion::operator+(const Quaternion& other) 
	{
		return Quaternion(other.getImg().getX()+img.getX(), other.getImg().getY()+img.getY(), other.getImg().getZ()+img.getZ(), other.getRe()+re);
	}

	Quaternion Quaternion::conjugate() 
	{
		return Quaternion(-img.getX(), -img.getY(), -img.getZ(), re);
	}

	Quaternion operator*(const Quaternion& quaternion1, const Quaternion& quaternion2) 
	{
		Scalar _re = (quaternion1.getRe() * quaternion2.getRe()) - dot(quaternion1.getImg(), quaternion2.getImg());
		Vector3D _img = (quaternion1.getRe() * quaternion2.getImg()) + (quaternion2.getRe() * quaternion1.getImg()) + cross(quaternion1.getImg(), quaternion2.getImg());
		return Quaternion(_img, _re);
	}

	Scalar Quaternion::norm() 
	{
		return std::sqrt(this->squaredNorm());
	}

	Scalar Quaternion::squaredNorm() 
	{
		return (this->getRe() * this->getRe()) + this->getImg().squaredNorm();
	}

}


