#ifndef PLANE_HPP
#define PLANE_HPP

#include <math.h>
#include "Line3D.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;

namespace TRTK
{
	class Plane
	{
	public:
//typedef double value_type;

		Plane(const Plane &);														///< Copy -constructs.
		Plane(const Vector3d&, const Vector3d&, const Vector3d&);		///< Constructs a plane from tree points.
		Plane(const Line3D &, const Line3D &);									///< Constructs a plane from two lines.
		Plane(const Vector3d&, const Vector3d&);							///< Constructs a plane from point and normalvector.
		Plane(const Line3D &, const Vector3d&);								///< Constructs plane from line and point.

		Vector3d pointOnPlane;
		Vector3d normalVector;

		Vector3d  getPoint() const;						///< Returns a point on the  plane.
		Vector3d   getNormalVector() const;				///< Returns a normal vector for a plane.

		void setPointOnPlane(const Vector3d point)  ;		 
		void setNormalVector(const Vector3d vector)  ;		 
	//	void set(const Vector3d point, const Vector3d vector) const;
		bool isParallel(const Plane &) const;	///< Returns is planes are parallel
		bool isIntersecting(const Plane &);	///<Returns if a Plane intersecting another plane


		double getDistancePoint(const Vector3d&) const;	///< Returns a distance between a point and plane.
		double getDistansePlane(const Plane &) const;		///< Returns a distance between two planes.
		

		Line3D  getIntersectingLine(const Plane &) const;	///< Returns a line, that intersecting to the plane
		double getAngleBetweenPlanes(const Plane &) const;	///< Returns an angle between two planes
		double getAngleBetweenPlaneAndLine(const Line3D &) const;	///< Returns an angle between the plane and a line
		Vector3d   intersectionLineAndPlane(const Line3D &) const;	///< Returns a vector, that intersecting to the plane
		//void Ausgleichsebene(const Vector3d&, const Vector3d&);	///<
	};



 
} //End of the namespace TRTK

#endif

