#ifndef LINE3D_HPP
#define LINE3D_HPP

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "TRTK/ErrorObj.hpp"


/** \tparam T scalar type
*
* \brief A generic coordinate class.
*
* Line3D represents an ordinary line that consists of one point on the line and
* normal vector. This line is represented in 3 dimensions.
* Its elements are of Vector3d \c T and can be freely chosen,
* thus also allowing the use of classes with arbitrary precision (like CLN or
* GMP), for instance.
*
*
*
* Here are some examples:
*
* \code
*
* using namespace TRTK;
*
* Line3D<T> line1(Coordinate<double> point(1, 0), Coordinate<double> direction(1, 0));
* Line3D<T> line2(Line3D<T> line1);
*
* Line3D<T> line3 = line1 * 3 ;
*
* !!cout << c3        << endl
*  !!    << c3.norm() << endl
*  !!    << (c3, 1)   << endl;
*
* !!\endcode
*
*!! Output:
*
* !!\code
*
* !!(3, 4)
* !!5
* !!(3, 4, 1)
*
* !!\endcode
*

*TODO
* typedef Line3D<double> CoordinateD;
* typedef Coordinate<int> CoordinateI;
*
*/
	using namespace Eigen;

	namespace TRTK
	{

		class Line3D
		{
		public:



			Vector3d point;
			Vector3d direction;


			Line3D();											///< Constructs a zero line. It will produce one line with the coordinate from the point (0,0,0)  and direction (0,0,1).
			Line3D(const Vector3d&, const Vector3d&);  ///< Constructs a line, which has a given coordinate and given direction.

			Line3D(const Line3D &line);						///< Copies -constructs.


			Vector3d  getPoint() const;						///< Returns the coordinate point on the line.
			Vector3d   getDirection() const;					///< Returns the direction of the line.

			bool operator==(const Line3D &line1) const;				///< Compares the lines.  
			bool operator!=(const Line3D &line1) const;				///< Compares the lines. 

			Line3D  operator *(const double &scalar) const;			///< Component-wnormalizeise multiplication with a scalar.
			Line3D  operator /(const double &scalar) const;			///< Component-wise division with a scalar.

			void setPoint(const Vector3d&);					///< Changes the value of a point of the line. 
			void setPoint(const double&, const double&, const double&);			///< Changes the value of the components of a point of the line. . 

			void setDirection(const Vector3d&);				///< Changes the value of the direction of the line. 
			void setDirection(const double&, const double&, const double&);		///< Changes the value of the components of the direction of the line. 
			void set(const Vector3d&, const Vector3d&);	///< Changes the values of the point and direction of the line.  


			bool isNormalized() const;								///< Returns true if the direction is normalized.
			Vector3d normalize() const;						///< Returns a normalized vector for line.
			bool  isParallel(const Line3D&) const;					///< Returns true if lines are parallel.
			bool isIntersecting(const Line3D&) const;				///< Returns true if the lines intersect.
			bool   isPointOnTheLine(const Vector3d&) const;	///<  Returns true if a point lies on the line.
			double getAngleBetweenLines(const Line3D&) const;		///< Finds an angle between the lines 
			double  getMinDistance(const Line3D&) const;			///< Returns a minimal distance between two lines.
			double  getAngleBetweenVectorAndLine(const Vector3d&) const;			///< Calculates the angle between the line and a vector.
			double getAngleBetweenVectors(const Vector3d&) const;					///< Calculates the angle between two lines.
			Vector3d  getMinDistancePoint(const Line3D  &line2) const;			///< Calculates the equidistant point between two lines  
			Vector3d   intersectionLines(const Line3D &) const;					///< Finds the point where lines intersect.
			Line3D   getOrthogonal() const;											///< Calculates an orthogonal vector from the line. 
			double distancePoint2Line(const Vector3d&) const;						///< Calculates the distance between the line and a point.
			Vector3d  getProjectionPoint2Line(const Vector3d&) const;			///< Returns the perpendicular of one Point to the plane.
			Vector3d intersectionPoint(const Vector3d&) const;				///< Returns the point, where the plain and point intersect.
			//	void  ausgleichsgerade(const Coordinate<Vector3d> aPointList, const Vector3d &vLineNorm, const  Vector3d &vLinePoint);

		private:


		};


		/////////////////////////////////////////////////////////////////////////////////////	 


}
	 
#endif 

