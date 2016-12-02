#include "TRTK/Plane.hpp"

namespace TRTK{
/** \tparam T scalar type
*
* \param [in] plane, which parameters will copy
*
*/
Plane::Plane(const Plane & plane)
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
Plane::Plane(const Vector3d& pointA, const Vector3d& pointB, const Vector3d& pointC)
{
	this->pointOnPlane = pointA;  //Ortsvektor
	Vector3d ab = pointA - pointB; // Richtungsvektor1
	Vector3d bc = pointB - pointC; // Richtungsvektor2
	Vector3d normalVector = ab.cross(bc);
	this->normalVector = normalVector;

}

/** \tparam T scalar type
*
* \param [in] line1 first line on the plane
* \param [in] line2 second line on the plane
*/
Plane::Plane(const Line3D & line1, const Line3D & line2)
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
Plane::Plane(const Vector3d& point, const Vector3d& normalvector)
{
	pointOnPlane = point;  //Ortsvektor
	normalVector = normalvector;
}

/** \tparam T scalar type
*
* \param [in] line one line on the plane
* \param [in] pointA point on the plane
*/
Plane::Plane(const Line3D & line, const Vector3d& pointA)
{
	//	//Aus einem Punkt und einer Gerade lassen sich schnell drei Punkte erstellen.Ein Punkt ist uns schon durch A gegeben. Die anderen beiden wählen wir uns aus der Gerade.
	this->pointOnPlane = pointA;  //Ortsvektor
	Vector3d pointB = line.point + 1 * line.direction;
	Vector3d pointC = line.point + 2 * line.direction;

	Vector3d AB = pointA - pointB; // Richtungsvektor1
	Vector3d BC = pointB - pointC; // Richtungsvektor2

	Vector3d normalVector = AB.cross(BC);
	this->normalVector = normalVector;

}


/** \tparam T scalar type
*
* \param   distance from [in] point  to the plane \c *this will be calculated
*
* \return a calculated distance
*/
double Plane::getDistancePoint(const Vector3d& point) const{
	// lege eine Gerade durch den Punkt und suche den Durchstoeungspunkt
	Line3D  line2(point, normalVector.normalized());
	Vector3d vHelpPoint = intersectionLineAndPlane(line2);
	// bilde Differenz von vHelpPoint und vPoint
	Vector3d  vDiff = vHelpPoint - point;
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
double Plane::getDistansePlane(const Plane & plane2) const{
	//Hesse-Normalform bestimmen:
	Vector3d m_result1 = pointOnPlane.cwiseProduct(normalVector);
	double result = (m_result1.x() + m_result1.y() + m_result1.z()) * (-1);
	double normalVector1 = std::sqrt(normalVector.norm());
	double normalVector2 = std::sqrt(plane2.normalVector.norm());
	Vector3d m_point2(0, 0, result / normalVector.z());
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
bool Plane::isIntersecting(const Plane & plane2) {
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
bool Plane::isParallel(const Plane & plane2) const{
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
Line3D  Plane::getIntersectingLine(const Plane & plane2) const{
	// Wir haben zwei Ebenen in Paramatherform

	//Der Richtungsvektor der Geraden berechnet sich dann durch:
	Vector3d norm_line = plane2.normalVector.cross(normalVector);
	//Wir benötigen jetzt noch den Stützvektor. 
	//Dazu formen wir, wie oben erwähnt, die Ebenendarstellungen von der Normalenform in die allgemeine Ebenenform  




	Vector3d m_result2 = plane2.pointOnPlane.cwiseProduct(normalVector);
	double result2 = (m_result2.x() + m_result2.y() + m_result2.z());

	//Ein Richtungsvektor ergibt sich daher aus dem Vektorprodukt der beiden Normalenvektoren.
	Vector3d normalSchnitt = normalVector.cross(plane2.normalVector);
	double normalVector1 = normalVector.norm();
	double normalVector2 = plane2.normalVector.norm();
	///TODO !!!!!!!

	return Line3D(normalSchnitt, m_result2);
}


/** \tparam T scalar type
*
* \param the angle between [in] plane  and  \c *this will be calculated
*
* \return the angle between thetwo plaines
*/
double Plane::getAngleBetweenPlanes(const Plane & plane2) const{
	Vector3d sum = plane2.normalVector.cwiseProduct(normalVector);
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
double Plane::getAngleBetweenPlaneAndLine(const Line3D & line2)const{
	Vector3d sum = line2.direction.cwiseProduct(normalVector);
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
Vector3d  Plane::intersectionLineAndPlane(const Line3D  & line2) const{
	// gucken, dass Line nicht parallel zu Ebene liegt
	if (line2.direction.dot(normalVector) != 0){
		double dLambda = pointOnPlane.dot(normalVector) - line2.point.dot(line2.direction) / (normalVector.dot(line2.direction));
		Vector3d vIntersection = (pointOnPlane + dLambda * normalVector);
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

Vector3d  Plane::getPoint() const{
	return pointOnPlane;
}

///< Returns a normal vector for a plane.

Vector3d   Plane::getNormalVector() const{
	return normalVector;
}

void Plane::setPointOnPlane(const Vector3d point)  {
	this->pointOnPlane = point;
}

void Plane::setNormalVector(const Vector3d vector)  {
	this->normalVector = vector;
}
}