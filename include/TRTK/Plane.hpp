#ifndef PLANE_HPP
#define PLANE_HPP

#include <TRTK/Coordinate.hpp>
#include <math.h>
#include <Line3D.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;

namespace TRTK
{
	template <class T>
	class Plane
	{
	public:
		typedef T value_type;

		Plane<T>(const Plane<T>&);														///< Copy -constructs.
		Plane<T>(const Coordinate<T>&, const Coordinate<T>&, const Coordinate<T>&);		///< Constructs a plane from tree points.
		Plane<T>(const Line3D<T>&, const Line3D<T>&);									///< Constructs a plane from two lines.
		Plane<T>(const Coordinate<T>&, const Coordinate<T>&);							///< Constructs a plane from point and normalvector.
		Plane<T>(const Line3D<T>&, const Coordinate<T>&);								///< Constructs plane from line and point.

		Coordinate<T> pointOnPlane;
		Coordinate<T> normalVector;

		Coordinate<T>  getPoint() const;						///< Returns a point on the  plane.
		Coordinate<T>   getNormalVector() const;				///< Returns a normal vector for a plane.

		void setPointOnPlane(const Coordinate<T> point) const;		 
		void setNormalVector(const Coordinate<T> vector) const;		 
	//	void set(const Coordinate<T> point, const Coordinate<T> vector) const;
		bool isParallel(const Plane<T>&) const;	///< Returns is planes are parallel
		bool isIntersecting(const Plane<T>&);	///<Returns if a Plane intersecting another plane


		double getDistancePoint(const Coordinate<T>&) const;	///< Returns a distance between a point and plane.
		double getDistansePlane(const Plane<T>&) const;		///< Returns a distance between two planes.
		

		Line3D<T> getIntersectingLine(const Plane<T>&) const;	///< Returns a line, that intersecting to the plane
		double getAngleBetweenPlanes(const Plane<T>&) const;	///< Returns an angle between two planes
		double getAngleBetweenPlaneAndLine(const Line3D<T>&) const;	///< Returns an angle between the plane and a line
		Coordinate<T>   intersectionLineAndPlane(const Line3D<T>&) const;	///< Returns a vector, that intersecting to the plane
		//void Ausgleichsebene(const Coordinate<T>&, const Coordinate<T>&);	///<
	};


	/** \tparam T scalar type
	*
	* \param [in] plane, which parameters will copy
	*
	*/
	template <class T> //Three points
	Plane<T>::Plane(const Plane<T>& plane)
	{
		this->pointOnPlane = plane.pointOnPlane;
		this->normalVector = plane.normalVector;

	}

	/** \tparam T scalar type
	*
	* \param [in] pointA first point on the plane
	* \param [in] pointB second point on the plane
	* \param [in] pointC third point on the plane
	*/
	template <class T> //Three points
	Plane<T>::Plane(const Coordinate<T>& pointA, const Coordinate<T>& pointB, const Coordinate<T>& pointC)
	{
		this->pointOnPlane = pointA;  //Ortsvektor
		Coordinate<T> ab = pointA - pointB; // Richtungsvektor1
		Coordinate<T> bc = pointB - pointC; // Richtungsvektor2
		Coordinate<T> normalVector = ab.cross(bc);
		this->normalVector = normalVector;

	}

	/** \tparam T scalar type
	*
	* \param [in] line1 first line on the plane
	* \param [in] line2 second line on the plane
	*/
	template <class T> //Two Lines
	Plane<T>::Plane(const Line3D<T>& line1, const Line3D<T>& line2)
	{
		//Gegeben sind zwei Geraden, die nicht windschief sind, sondern sich schneiden oder parrallel zu einander sind
		if (line1.isParallel(line2) || line1.isIntersecting(line2)){
			//	Man wählt sich den Schnittpunkt der beiden Geraden als Stützvektor
			//und fügt beide Richtungsvektoren der Gleichungen als Spannvektoren hinzu und erhällt die Paramerterform der Ebene.
			this->pointOnPlane = line1.point;
			this->normalVector = line1.direction.cross(line2.direction);
		}
		else{
			//Geraden sind windschief
			std::stringstream os;
			os << "***\nFunction: " << __FUNCTION__ << " failed in File: " << __FILE__ << " Line: " << __LINE__ << "Lines are croocked 0\n";
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			error.setErrorMessage(os.str());
			throw error;
		}
	}

	/** \tparam T scalar type
	*
	* \param [in] point one point on the plane
	* \param [in] normalvector normal vector of a plane
	*/
	template <class T> //Point and normalvector
	Plane<T>::Plane(const Coordinate<T>& point, const Coordinate<T>& normalvector)
	{
		pointOnPlane = point;  //Ortsvektor
		normalVector = normalvector;
	}

	/** \tparam T scalar type
	*
	* \param [in] line one line on the plane
	* \param [in] pointA point on the plane
	*/
	template <class T>// One line and one point
	Plane<T>::Plane(const Line3D<T>& line, const Coordinate<T>& pointA)
	{
		//	//Aus einem Punkt und einer Gerade lassen sich schnell drei Punkte erstellen.Ein Punkt ist uns schon durch A gegeben. Die anderen beiden wählen wir uns aus der Gerade.
		this->pointOnPlane = pointA;  //Ortsvektor
		Coordinate<T> pointB = line.point + 1 * line.direction;
		Coordinate<T> pointC = line.point + 2 * line.direction;

		Coordinate<T> AB = pointA - pointB; // Richtungsvektor1
		Coordinate<T> BC = pointB - pointC; // Richtungsvektor2

		Coordinate<T> normalVector = AB.cross(BC);
		this->normalVector = normalVector;

	}
	 

	/** \tparam T scalar type
	*
	* \param   distance from [in] point  to the plane \c *this will be calculated
	*
	* \return a calculated distance
	*/
	template <class T>
	double Plane<T>::getDistancePoint(const Coordinate<T>& point) const{
		// lege eine Gerade durch den Punkt und suche den Durchstoeungspunkt
		Line3D<T> line2(point, normalVector.normalized());
		Coordinate<T> vHelpPoint = intersectionLineAndPlane(line2);
		// bilde Differenz von vHelpPoint und vPoint
		Coordinate<T>  vDiff = vHelpPoint - point;
		// bilde den Betrag und damit die Entfernung
		double dist = vDiff.norm();
		return dist;
	}

	/** \tparam T scalar type
	*
	* \param distance from [in] plane2 to the plane \c *this will be calculated
	*
	* \return a calculated distance
	*/
	template <class T>
	double Plane<T>::getDistansePlane(const Plane<T>& plane2) const{
		//Hesse-Normalform bestimmen:
		Coordinate<T> m_result1 = pointOnPlane*normalVector;
		double result = (m_result1.x() + m_result1.y() + m_result1.z()) * (-1);
		double normalVector1 = std::sqrt(normalVector.norm());
		double normalVector2 = std::sqrt(plane2.normalVector.norm());
		Coordinate<T> m_point2(0, 0, result / normalVector.z());
		//Punkt in Hesse-Normalform einsetzen:
		double dist = normalVector.x()*m_point2.x() - normalVector.y()*m_point2.x() - normalVector.z()*m_point2.z() - result;
		return dist;
	}

	/** \tparam T scalar type
	*
	* \param  [in] plane intesects with \c *this
	*
	* \return true if planes interect each other
	*/
	template <class T>
	bool Plane<T>::isIntersecting(const Plane<T>& plane2) {
		if (getDistansePlane(plane2) == 0){
			return true;
		}
		else {
			return false;
		}
	}

	/** \tparam T scalar type
	*
	* \param   [in] plane2 parallel with \c *this
	*
	* \return true if plane are parallel
	*/
	template <class T>
	bool Plane<T>::isParallel(const Plane<T>& plane2) const{
		double r1 = normalVector.x() / plane2.normalVector.x();
		double r2 = normalVector.y() / plane2.normalVector.y();
		double r3 = normalVector.z() / plane2.normalVector.z();
		if (r1 == r2 && r2 == r3){
			return true;
		}
		else{
			return false;
		}
	}

	/** \tparam T scalar type
	*
	* \param [in] plane2 plane to be compared with \c *this
	*
	* \return the intersection line between two planes
	*/
	template <class T>
	Line3D<T> Plane<T>::getIntersectingLine(const Plane<T>& plane2) const{
	// Wir haben zwei Ebenen in Paramatherform

		//Der Richtungsvektor der Geraden berechnet sich dann durch:
		Coordinate<T> norm_line = plane2.normalVector.cross(normalVector);
		//Wir benötigen jetzt noch den Stützvektor. 
		//Dazu formen wir, wie oben erwähnt, die Ebenendarstellungen von der Normalenform in die allgemeine Ebenenform  



		double result1 = (m_result1.x() + m_result1.y() + m_result1.z());
		Coordinate<T> m_result2 = plane2.pointOnPlane*plane2.normalVector;
		double result2 = (m_result2.x() + m_result2.y() + m_result2.z());

		//Ein Richtungsvektor ergibt sich daher aus dem Vektorprodukt der beiden Normalenvektoren.
		Coordinate<T> normalSchnitt = normalVector.cross(plane2.normalVector);
		double normalVector1 = normalVector.norm();
		double normalVector2 = plane2.normalVector.norm();
		///TODO !!!!!!!
		return normalSchnitt;
	}


	/** \tparam T scalar type
	*
	* \param the angle between [in] plane  and  \c *this will be calculated
	*
	* \return the angle between thetwo plaines
	*/
	template <class T>
	double Plane<T>::getAngleBetweenPlanes(const Plane<T>& plane2) const{
		Coordinate<T> sum = plane2.normalVector * normalVector;
		double CoordinSumme = sum.x() + sum.y() + sum.z();
		double ret = CoordinSumme / (normalVector.norm() *plane2.normalVector.norm());
		return (ret);
	}
	/** \tparam T scalar type
	*
	* \param the angle between [in] line2  and  \c *this will be calculated
	*
	* \return the angle between the point and the line
	*/
	template <class T>double Plane<T>::getAngleBetweenPlaneAndLine(const Line3D<T>& line2)const{
		Coordinate<T> sum = line2.direction * normalVector;
		double CoordinSumme = sum.x() + sum.y() + sum.z();
		double ret = CoordinSumme / (normalVector.norm() *line2.direction.norm());
		return ret;
	}
	/** \tparam T scalar type
	*
	* \param   the intersection line between [in] line2  and  \c *this will be calculated
	*
	* \return the the intersection line between the plane and the line
	*/
	template <class T>
	Coordinate<T>   Plane<T>::intersectionLineAndPlane(const Line3D<T> & line2) const{
		// gucken, dass Line nicht parallel zu Ebene liegt
		if (line2.direction.dot(normalVector) != 0){
		double dLambda = pointOnPlane.dot(normalVector) - line2.point.dot(line2.direction) / (normalVector.dot(line2.direction));
		Coordinate<double> vIntersection = (pointOnPlane + dLambda * normalVector);
		return vIntersection;
		}
		else{
			std::stringstream os;
			os << "***\nFunction: " << __FUNCTION__ << " failed in File: " << __FILE__ << " Line: " << __LINE__ << "Line and Plane are parallel 0\n";
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			error.setErrorMessage(os.str());
			throw error;
		}
	}

	///< Returns a point on the  plane.
	template <class T>
	Coordinate<T>  Plane<T>::getPoint() const{
		return pointOnPlane; 
	}

	///< Returns a normal vector for a plane.
	template <class T>
	Coordinate<T>   Plane<T>::getNormalVector() const{
		return normalVector; 
	}
	template <class T>
	void Plane<T>::setPointOnPlane(const Coordinate<T> point) const{
		pointOnPlane = point; 
	}
	template <class T>
	void Plane<T>::setNormalVector(const Coordinate<T> vector) const{
		normalVector = vector; 
	}

	//template <class T>
	//void Plane<T>::set(const Coordinate<T> point, const Coordinate<T> vector) const{
	//	normalVector = vector;
	//	pointOnPlane = point;
	//}
} //End of the namespace TRTK

#endif

