#ifndef LINE3D_HPP
#define LINE3D_HPP

#include <TRTK/Coordinate.hpp>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "TRTK/ErrorObj.hpp"

using namespace Eigen;

/** \tparam T scalar type
*
* \brief A generic coordinate class.
*
* Line3D represents an ordinary line that consists of one point on the line and
* normal vector. This line is represented in 3 dimensions.
* Its elements are of Coordinate<T> \c T and can be freely chosen,
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


namespace TRTK
{
	template <class T>
	class Line3D
	{
	public:
		typedef T value_type;
		typedef TRTK::Coordinate<T> data_type;

		Coordinate<T> point;
		Coordinate<T> direction;

		Line3D<T>();											///< Constructs a zero line. It will produce one line with the coordinate from the point (0,0,0)  and direction (0,0,1).
		Line3D<T>(const Coordinate<T>&, const Coordinate<T>&);  ///< Constructs a line, which has a given coordinate and given direction.
		Line3D<T>(const Line3D<T> &line);						///< Copies -constructs.


		Coordinate<T>  getPoint() const;						///< Returns the coordinate point on the line.
		Coordinate<T>   getDirection() const;					///< Returns the direction of the line.

		bool operator==(const Line3D &line1) const;				///< Compares the lines.  
		bool operator!=(const Line3D &line1) const;				///< Compares the lines. 

		Line3D<T> operator *(const T &scalar) const;			///< Component-wnormalizeise multiplication with a scalar.
		Line3D<T> operator /(const T &scalar) const;			///< Component-wise division with a scalar.

		void setPoint(const Coordinate<T>&);					///< Changes the value of a point of the line. 
		void setPoint(const T&, const T&, const T&);			///< Changes the value of the components of a point of the line. . 

		void setDirection(const Coordinate<T>&);				///< Changes the value of the direction of the line. 
		void setDirection(const T&, const T&, const T&);		///< Changes the value of the components of the direction of the line. 
		void set(const Coordinate<T>&, const Coordinate<T>&);	///< Changes the values of the point and direction of the line.  


		bool isNormalized() const;								///< Returns true if the direction is normalized.
		Coordinate<T> normalize() const;						///< Returns a normalized vector for line.
		bool  isParallel(const Line3D&) const;					///< Returns true if lines are parallel.
		bool isIntersecting(const Line3D&) const;				///< Returns true if the lines intersect.
		bool   isPointOnTheLine(const Coordinate<T>&) const;	///<  Returns true if a point lies on the line.
		double getAngleBetweenLines(const Line3D&) const;		///< Finds an angle between the lines 
		double  getMinDistance(const Line3D&) const;			///< Returns a minimal distance between two lines.
		double  getAngleBetweenVectorAndLine(const Coordinate<T>&) const;			///< Calculates the angle between the line and a vector.
		double getAngleBetweenVectors(const Coordinate<T>&) const;					///< Calculates the angle between two lines.
		Coordinate<T>  getMinDistancePoint(const Line3D<T> &line2) const;			///< Calculates the equidistant point between two lines  
		Coordinate<T>   intersectionPoint(const Line3D<T>&) const;					///< Finds the point where lines intersect.
		Line3D<T>  getOrthogonal() const;											///< Calculates an orthogonal vector from the line. 
		double distancePoint2Line(const Coordinate<T>&) const;						///< Calculates the distance between the line and a point.
		Coordinate<T>  getProjectionPoint2Line(const Coordinate<T>&) const;			///< Returns the perpendicular of one Point to the plane.

		void  ausgleichsgerade(const Coordinate<Coordinate<T>> aPointList, const Coordinate<T> &vLineNorm, const  Coordinate<T> &vLinePoint);

	private:
		data_type m_data;
	};


	/////////////////////////////////////////////////////////////////////////////////////	 
	template <class T>
	double  test(const int t){
		return t;
	}

	template <class T>
	Line3D<T>::Line3D()
	{
		point = Coordinate<T>(0, 0, 0);
		direction = Coordinate<T>(0, 0, 1);
	}

	/** \tparam T scalar type
	*
	* \param [in] point on the line
	* \param [in] direction of the line
	*/
	template <class T>
	Line3D<T>::Line3D(const Coordinate<T> &point, const Coordinate<T> &direction)
	{
		if (direction.norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}
		else{

			this->point = point;
			this->direction = direction;
		}
	}

	/** \tparam T scalar type
	*
	* \param [in] line, which parameters will copy
	*/
	template <class T>
	Line3D<T>::Line3D(const Line3D<T> &line)
	{
		point = line.point;
		direction = line.direction;
	}


	/** \tparam T scalar type
	*
	* \param [in] line2 to be compared to \c *this
	*
	* \return true if lines overlap each other
	*/
	template <class T>
	bool Line3D<T>::operator==(const Line3D & line2) const
	{
		return  (isParallel(line2) && line2.isPointOnTheLine(point));
	}


	/** \tparam T scalar type
	*
	* \param [in] line1 to be compared to \c *this
	*
	* \return true if lines do not overlap
	*         each other.
	*/
	template <class T>
	bool Line3D<T>::operator!=(const Line3D & line1) const
	{
		if (isParallel(line1)){
			if (line1.isPointOnTheLine(point)){
				return false;
			}
			else{
				return true;
			}

		}
		else{
			return true;
		}
	}

	/** \tparam T scalar type
	*
	* \param [in] scalar will be multiplied component-wise with  \c *this
	*
	* \return Line3D<T>
	*/
	template <class T>
	Line3D<T> Line3D<T>::operator*(const T & scalar) const
	{
		if (scalar == 0){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		return Line3D<T>(point, (Coordinate<T>(direction.x() * scalar, direction.y() * scalar, direction.z() * scalar)));

	}




	/** \tparam T scalar type
	*
	* \param  \c *this directions components will be divied component-wise by [in] scalar
	*
	* \return Line3D<T>
	*/
	template <class T>
	Line3D<T> Line3D<T>::operator/(const T & scalar) const
	{
		if (scalar == 0){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		return Line3D<T>(point, (Coordinate<T>(direction.x() / scalar, direction.y() / scalar, direction.z() / scalar)));

	}

	/** \tparam T scalar type
	*
	*
	*
	* \return true if the direction is normalized
	*
	*/
	template <class T>
	bool Line3D<T>::isNormalized()const{
		if (direction.norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}
		if (direction.norm() == 1){
			return true;
		}
		else {
			return false;
		}
	}

	/** \tparam T scalar type
	*
	*
	*
	* \return the vector norm
	*/
	template <class T>
	Coordinate<T> Line3D<T>::normalize() const{
		return direction * (1 / direction.norm());
	}


	// Test if the vectors are parallel 

	/** \tparam T scalar type
	*
	* \param [in] line2 line to be compared with \c *this
	*
	* \return true if vectors are parallel
	*/
	template <class T>
	bool Line3D<T>::isParallel(const Line3D &line2) const
	{
		double r1 = direction.x() / line2.direction.x();
		double r2 = direction.y() / line2.direction.y();
		double r3 = direction.z() / line2.direction.z();
		if (r1 == r2 && r2 == r3){
			return true;
		}
		else{
			return false;
		}
	}



	/** \tparam T scalar type
	*
	* \param [in] line2 line to be compared with \c *this
	*
	* \return true if a point lies on the line
	*/
	template <class T>
	bool  Line3D<T>::isPointOnTheLine(const Coordinate<T> & point2) const
	{
		if (point2.size() < 3){
			ErrorObj error("The point hasn´t appropriate dimension \n");
			error.setClassName("Line3D");
			error.setFunctionName("isPointOnTheLine");
			throw error;
		}
		else{
			Coordinate<double> m_point = point2 - point;
			Coordinate<double>  diff_point = m_point / direction;

			if ((diff_point.x() == diff_point.y()) && (diff_point.z() == diff_point.y())){
				return true;
			}
			else {
				return false;
			}
			return false;
		}
	}

	/** \tparam T scalar type
	*
	* \param [in] line2 line to be compared with \c *this
	*
	* \return a calculated  angle between two lines
	*/
	template <class T>
	double Line3D<T>::getAngleBetweenLines(const Line3D & line2) const
	{
		//For an angle between two lines  their  vectors are essential
		//It´s enough to  find  an angle between both  vectors to find the angle between the lines.
		if ((line2.direction.norm() == 0) || (direction.norm() == 0))
		{
			ErrorObj error("The direction´s norm is null \n");;
			error.setClassName("Line3D");
			error.setFunctionName("getAngleBetweenLines");
			throw error;
		}

		if (isParallel(line2) && getMinDistance(line2) == 0){
			return 0;

		}

		else if (!isIntersecting(line2) && !isParallel(line2)){
			ErrorObj error("The lines are not intersecting  \n");;
			error.setClassName("Line3D");
			error.setFunctionName("getAngleBetweenLines");
			throw error;

		}

		else{
			Coordinate<T> sum = line2.direction * direction;
			double coordinSumme = sum.x() + sum.y() + sum.z();
			return acos(coordinSumme / (direction.norm() *line2.direction.norm()));

		}


	}



	/** \tparam T scalar type
	*
	* \param The angle between [in] Vector2  and \c *this will be calculated
	*
	* \return the angle between the line and the vector
	*/
	template <class T>
	double Line3D<T>::getAngleBetweenVectorAndLine(const Coordinate<T> & vector2) const
	{
		if ((vector2.norm() == 0) || (direction.norm() == 0))
		{
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		return acos((vector2.dot(direction) / (vector2.norm()*direction.norm())));

	}

	/** \tparam T scalar type
	*
	* \param if [in] line2 intesects with \c *this
	*
	* \return true if lines interect each other
	*/
	template <class T>
	bool Line3D<T>::isIntersecting(const Line3D & line2) const
	{
		if (isParallel(line2)){
			return false;
		}
		else {
			if (getMinDistance(line2) == 0){
				return true;

			}
			else{
				return false;
			}
		}

	}


	/** \tparam T scalar type
	*
	* \param [in] line2 is the second line
	*
	* \return the smallest distance between two lines
	*/
	template <class T>
	double  Line3D<T>::getMinDistance(const Line3D & line2)const{
		// Im folgenden wird der Punkt mit dem kleinsten Abstand von zwei (vielleicht windschiefen) Geraden berechnet.
		// Die Geraden sind:
		//		vLine1Point + vLambda(0) * vLine1Direction
		//		vLine2Point + vLambda(1) * vLine2Direction
		// Berechnung des Schnittpunkts der beiden Geraden.
		// Wenn das Gleichungssystem nicht lösbar ist, sind die Geraden parallel oder windschief,
		// wenn das Gleichungssystem eindeutig lösbar ist, schneiden die Geraden sich in einem Punkt,
		// wenn das Gleichungssystem unendlich viele Lösungen hat, liegen die Geraden auf einander.
		//
		//	(1) point + vLambda(0) * direction = line2.point + vLambda(1) * line2.direction
		//
		// das lässt sich wie folgt um formen:
		//
		//	(2)	point - line2.point = vLambda(1) * line2.direction - vLambda(0) * direction
		//
		// in Vektor/Matrix-Schreibweise (Matrix * vLambda = X):
		//	(3)
		//  ( -direction(0)  line2.direction(0) )   (vLambda(0))   (point(0) - line2.point(0))
		//	( -direction(1)  line2.direction(1) ) * (vLambda(1)) = (point(1) - line2.point(1))
		//	( -direction(2)  line2.direction(2) )			   	   (point(2) - line2.point(2))
		//
		// Multiplizieren von der linken Seite mit der transponierten (~) Matrix
		//  (4)
		//	( -direction(0)  -direction(2)  -direction(2) )		( -direction(0)  line2.direction(0) )   (vLambda(0))   ( -direction(0)  -direction(2)  -direction(2) )		(point(0) - line2.point(0))
		//	(  line2.direction(0)   line2.direction(1)   line2.direction(2) )	*	( -direction(1)  line2.direction(1) ) * (vLambda(1)) = (  line2.direction(0)   line2.direction(1)   line2.direction(2) )	*	(point(1) - line2.point(1))
		//	 
		Coordinate<T> vTmp2, vTmp;

		Eigen::Vector2d vLambda(0, 0);
		Eigen::Matrix3d mEquations(3, 3);
		mEquations.setIdentity();
		double dAngle = 0;
		Eigen::MatrixXd mTemp(3, 2);

		mTemp(0, 0) = -direction.x();
		mTemp(1, 0) = -direction.y();
		mTemp(2, 0) = -direction.z();
		mTemp(0, 1) = line2.direction.x();
		mTemp(1, 1) = line2.direction.y();
		mTemp(2, 1) = line2.direction.z();


		//~mTemp * mTemp * Lambda = ~mTemp * (point - line2.point)
		Eigen::MatrixXd mMatrix = mTemp.transpose() * mTemp;

		// Pruefen von mMatrix auf Singularitaet
		bool IsSingular = true;


		double det = mMatrix.determinant();
		if (mMatrix.determinant() < 1.0e-12){

			IsSingular = true;
		}
		else{
			IsSingular = false;
		}

		if (IsSingular)	{ // Lines are parallel or a directions vector is null vector 
			if ((line2.direction.norm() == 0) || (direction.norm() == 0))
			{
				ErrorObj error;
				error.setClassName("Line3D");
				error.setFunctionName(__FUNCTION__);
				throw error;

			}
			else{ 	// Lines are parallel or identical

				// When two lines are parallel, every point on the first line has a minimum distanse to other line 

				vTmp2 = point - line2.point;
				if (vTmp2.norm() == 0)
				{
					return 0.0;

				}
				// Winkelbestimmung der Verbindung und des Richtungsvektors der ersten Geraden


				dAngle = getAngleBetweenVectorAndLine(vTmp2);
				// 	Bestimmung des Abstands durch Bildung des Betrages der durch Projektion entstandenen Strecke
				double ret = (vTmp2.norm() * sin(dAngle));
				return ret;

			}
		}
		else
		{
			//nicht singulär
			//gilt, wenn sie sich schneiden oder windschief sind.
			// Das Gleichungssystem wird gelöst
			// (1) von links mit der inversen (!) Matrix multiplizieren
			// 
			//	!mMatrix * mMatrix * vLambda = !mMatrix * ~mTemp * (point - line2.point)
			//
			// (2) da !mMatrix * mMatrix die Einheitsmatrix ist, erhält man folgende Gleichung:
			Coordinate < double> diffPoint = point - line2.point;
			Eigen::Vector3d diff(diffPoint.x(), diffPoint.y(), diffPoint.z());
			vLambda = mMatrix.inverse() *mTemp.transpose() *  diff;
			Coordinate<double> vpointOnLine1 = point + vLambda[0] * direction;
			Coordinate<double> vPointOnLine2 = line2.point + vLambda[1] * line2.direction;

			return (vpointOnLine1 - vPointOnLine2).norm();
		}
	}


	/** \tparam T scalar type
	*
	* \param [in] line2 is the second line
	*
	* \return the point in which two lines are intersect
	*/

	template <class T>
	Coordinate<T>   Line3D<T>::intersectionPoint(const Line3D<T> &line2) const
	{
		if (line2.direction.norm() == 0){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		// Wenn Geraden sich nicht schneiden
		if (!isIntersecting(line2)){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		else{
			/*
			g = p + t*r // p ist ein Punkt der Geraden, r die Richtung der Geraden.

			g1 = p1+t1*r1
			g2 = p2+t2*r2
			g1 = g2

			// und daraus ergibt sich:
			p1+t1*r1 = p2+t2*r2

			// oder ausgeschrieben und umgeformt:
			r1x*t1 - r2x*t2 = p2x-p1x
			r1y*t1 - r2y*t2 = p2y-p1y
			r1z*t1 - r2z*t2 = p2z-p1z
			*/
			//Coordinate<T> vTmp(direction.normalized().cross(line2.direction.normalized()));
			// Überbestimmte gleichungssystem 
			// 

			//Coordinate<T> vTmp(direction.normalized().cross(line2.direction.normalized()));
			Coordinate<T> vTmp((line2.point - point).x(), (line2.point - point).y(), (line2.point - point).z());
			Eigen::Matrix3d mEquations(3, 3);
			mEquations(0, 0) = direction.normalized().x();
			mEquations(1, 0) = direction.normalized().y();
			mEquations(2, 0) = direction.normalized().z();
			mEquations(0, 1) = -line2.direction.normalized().x();
			mEquations(1, 1) = -line2.direction.normalized().y();
			mEquations(2, 1) = -line2.direction.normalized().z();
			mEquations(0, 2) = vTmp.x();
			mEquations(1, 2) = vTmp.y();
			mEquations(2, 2) = vTmp.z();
			Eigen::Matrix2d xy(2, 2);
			xy(0, 0) = mEquations(0, 0);
			xy(1, 0) = mEquations(1, 0);
			xy(0, 1) = mEquations(0, 1);
			xy(1, 1) = mEquations(1, 1);

			Eigen::Matrix2d mEquations2 = xy.transpose() * xy;


			Coordinate<T> diff(line2.point - point);
			Eigen::Vector3d m_temp(diff.x(), diff.y(), diff.z());
			Eigen::Vector3d m_temp2 = mEquations.inverse() * m_temp;
			Coordinate<T> vLambda(m_temp2.x(), m_temp2.y(), m_temp2.z());
			// Der gesuchte Punkt liegt dann jetzt an der Stelle vLine2Point + vLambda(1) * vLine2Norm
			vTmp = point + vLambda.x() * direction.normalized() + (vLambda.z() * vTmp) / 2.0;

			// Den Punkt auf Line1, der den minimalen Abstand zu Line 2 hat in vPointOnLine1 zur�ckgeben
			//	vPointOnLine1 = vTmp;

			// R�ckgabewert ist der minimale Abstand der beiden Gerade
			double vTmpx = vTmp.x();
			double vTmpy = vTmp.y();
			double vTmpz = vTmp.z();
			return vTmp;


		}


	}




	/** \tparam T scalar type
	*
	*
	* \return direction value of the line
	*/
	template <class T>
	Coordinate<T>  Line3D<T>::getDirection() const
	{
		return direction;
	}

	/** \tparam T scalar type
	*
	*
	* \return a point of the line
	*
	*/
	template <class T>
	Coordinate<T>  Line3D<T>::getPoint() const
	{
		return point;
	}


	/** \tparam T scalar type
	*
	* \param [in] direction2 is a new direction value instead of \c *this
	*
	*/
	template <class T>
	void Line3D<T>::setDirection(const Coordinate<T> &direction2)
	{
		if (direction2.norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}
		direction = direction2;
	}


	/** \tparam T scalar type
	*
	* \param [in] x,  [in] y,  [in] z  are new components of the direction value \c *this
	*
	*/
	template <class T>
	void Line3D<T>::setDirection(const T &x, const  T &y, const T &z)
	{
		if (Coordinate<T>(x, y, z).norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}

		direction = Coordinate<T>(x, y, z);
	}


	//Set both values : point and direction

	/** \tparam T scalar type
	*
	* \param [in] point1 and [in] point2 are the new origin point of the line \c *this
	*
	*/
	template <class T>
	void Line3D<T>::set(const Coordinate<T> &point1, const Coordinate<T> &direction2)
	{
		if (direction2.norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}

		point = point1;
		direction = direction2;
	}



	/** \tparam T scalar type
	*
	* \param [in] point2 is the new origin point of the line \c *this
	*
	*/
	template <class T>
	void Line3D<T>::setPoint(const T &x, const T &y, const T &z)
	{
		point = Coordinate<T>(x, y, z);
		direction = point2 - point;
	}


	/** \tparam T scalar type
	*
	* \param [in] point2 is the new origin point of the line \c *this
	*
	*/
	template <class T>
	void Line3D<T>::setPoint(const Coordinate<T> &point2)
	{
		point = point2;
		direction = point2 - point;
	}


	/** \tparam T scalar type
	*
	*
	* \return the orthogonal vector of the line
	*/
	template <class T>
	Line3D<T> Line3D<T>::getOrthogonal() const
	{
		Coordinate<T> ret = direction.orthogonal();

		if (ret.norm() == 0){

			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}

		return Line3D<T>(point, ret);
	}

	///////////////////////////////////////////////////////////////////////////////
	/// Lot eines Punktes auf eine Gerade.
	/// \param		vPoint Punkt, von dem aus das Lot auf die Gerade gefällt werden soll.
	///	\return		Lotfußpunkt
	///	\author		<a href="mailto:fuente@hia.rwth-aachen.de">Matias de la Fuente</a>
	/// \date		2001-2007
	///	\note		MathUnits::BIB 3.4.4.11
	///////////////////////////////////////////////////////////////////////////////
	template <class T>
	Coordinate<T> Line3D<T>::getProjectionPoint2Line(const Coordinate<T> &vPoint) const
	{
		Line3D<T> v_line(vPoint, direction);
		return intersectionPoint(v_line);

	}
	/** \tparam T scalar type
	*
	* \param the minimal distance between [in] point2  and  \c *this will be calculated
	*
	* \return the minimal distance between the point and the line
	*/
	template <class T>
	double Line3D<T>::distancePoint2Line(const Coordinate<T> &point2) const
	{
		// Zuerst eine Ebene bilden, mit Normalenvektor=Geradenrichtung und durch den vPoint
		// dann diese Gerade mit der Ebene schneiden
		// und anschliessen den Abstand von dem Schnittpunkt zu dem
		// ubergebenen Punkt berechnen
		if (isPointOnTheLine(point2)){
			return 0;
		}
		else{
			Line3D<double> m_line(point2, direction);
			//Coordinate<T>  vIntersection( intersectionLineAndPlane(m_line));


			double dLambda = (m_line.point.dot(m_line.direction.normalized()) - point.dot(m_line.direction.normalized())) / (direction.dot(m_line.direction.normalized()));
			Coordinate<double> vIntersection = (point + dLambda * direction);
			return  (vIntersection - point2).norm();
		}

	}

	template <class T>

	/** \tparam T scalar type
	*
	* \param equidistant point will be calculated between [in] line2  and \c *this
	*
	* \return  the equidistant point between two lines
	*/
	Coordinate<T> Line3D<T>::getMinDistancePoint(const Line3D<T> &line2) const
	{
		// Um den kleinsten Abstand von zwei Geradenstuecken zu berechnen, muessen 5 Faelle betrachtet werden:
		// Entweder, ist einer der 4 Endpunkte derjenige Punkt, mit dem kleinsten Abstand, oder aber
		// es liegt irgendwo dazwischen.
		//
		// Fuer die Berechnung wird zuerst geguckt, wo die Beiden Geraden den kleinsten Abstand haben, wenn man
		// die geraden unendlich verl�ngert.
		// Dann muss geschaut werden, ob der gefundene Punkt kleinsten Abstandes innerhalb derbeiden Geradenst�cke liegt.
		// 
		// Nur wenn er ausserhalb liegt, dann ist einer der vier Endpunkte 


		// Im folgenden wird der Punkt mit dem kleinsten Abstand von zwei (vielleicht windschiefen) Geraden berechnet.
		// Die Geraden sind:
		//		vLine1Point + vLambda(0) * vLine1Norm
		//		vLine2Point + vLambda(1) * vLine2Norm

		Eigen::Matrix3d mEquations(3, 3);
		// Senkrechten Vektor auf beide Geraden bilden
		Coordinate<T> vTmp = direction.normalized().cross(line2.direction.normalized());
		// Jetzt muss der Punkt auf Line2 gefunden werden, der den kleinsten Abstand zu Line1 hat.
		//
		// dazu muss folgende Gleichung geloest werden:
		// der Vektor, der mit kleinstem Abstand Line1 und Line2 verbindet, laest sich auf zwei Arten bestimmen,
		// die dann gleichgesetzt werden
		//		vLambda(2) * (vLine1Norm x vLine2Norm) = (vLine2Point + vLambda(1) * vLine2Norm - (vLine1Point + vLambda(0) * vLine1Norm))
		//
		// daraus folgt das Gleichungssystem:
		//		mEquations * vLambda = vLine2Point - vLine1Point

		mEquations(0, 0) = direction.normalized().x();
		mEquations(1, 0) = direction.normalized().y();
		mEquations(2, 0) = direction.normalized().z();
		mEquations(0, 1) = -line2.direction.normalized().x();
		mEquations(1, 1) = -line2.direction.normalized().y();
		mEquations(2, 1) = -line2.direction.normalized().z();
		mEquations(0, 2) = vTmp.x();
		mEquations(1, 2) = vTmp.y();
		mEquations(2, 2) = vTmp.z();

		// Das Gleichungssystem wird geloest, indem beide Seiten des Gleichungssystems von links
		// mit der inversen Matrix multipliziert werden
		Coordinate<T> point_diff = (line2.point - point);

		Eigen::Vector3d m_temp = point_diff.toArray();
		//		m_temp(point_diff.x(), point_diff.y(), point_diff.z());
		Eigen::Vector3d m_temp2 = mEquations.inverse() * m_temp;

		Coordinate<T> vLambda(m_temp2.x(), m_temp2.y(), m_temp2.z());
		// Der gesuchte Punkt liegt dann jetzt an der Stelle vLine2Point + vLambda(1) * vLine2Norm
		vTmp = point + vLambda.x() * direction.normalized() + (vLambda.z() * vTmp) / 2.0;

		// Den Punkt auf Line1, der den minimalen Abstand zu Line 2 hat in vPointOnLine1 zur�ckgeben
		//	vPointOnLine1 = vTmp;

		// R�ckgabewert ist der minimale Abstand der beiden Gerade
		return vTmp;
	}

	template <class T>
	void Line3D<T>::ausgleichsgerade(const	Coordinate<Coordinate<T>> aPointList, const	 Coordinate<T> &vLineNorm, const Coordinate<T> &vLinePoint)
	{
		// �berpr�fe, ob genug Punkte �bergeben werden
		if (aPointList.size() < 3) {
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;

		}
		// �berpr�fen, ob genug (verschiedene) Punkte �bergeben worden sind
		unsigned int iCounter = 0;
		for (unsigned int i = 0; i < aPointList.size() - 1; i++) {
			if (aPointList[i].x() == aPointList[i + 1].x() && aPointList[i].y() == aPointList[i + 1].y() && aPointList[i].z() == aPointList[i + 1].z()) {
				iCounter++;
				if (aPointList.size() - iCounter < 3) {

					ErrorObj error;
					error.setClassName("Line3D");
					error.setFunctionName(__FUNCTION__);
					throw error;


				}
			}
		}

		Eigen::MatrixXd mA;
		mA.SetNull();

		// mean point
		double fXc = 0;
		double fYc = 0;
		double fZc = 0;
		for (unsigned int i = 0; i < aPointList.size(); i++)
		{
			fXc += aPointList[i].x();
			fYc += aPointList[i].y();
			fZc += aPointList[i].z();
		}
		// Mittelwert �ber alle Punkte bilden, dieser Punkt sollt dann auf der Ebene liegen.
		fXc = fXc / aPointList.size();
		fYc = fYc / aPointList.size();
		fZc = fZc / aPointList.size();

		// matrix mA is 3x3 symmetrical matrix 
		// with sum of squared distances between points and mean point on main diagonal
		// and sum of cross distances on off-diagonal
		for (unsigned int i = 0; i < aPointList.size(); i++)
		{
			mA(0, 0) += pow(aPointList[i](0) - fXc, 2);
			mA(1, 1) += pow(aPointList[i](1) - fYc, 2);
			mA(2, 2) += pow(aPointList[i](2) - fZc, 2);

			mA(0, 1) += (aPointList[i](0) - fXc)*(aPointList[i](1) - fYc);
			mA(0, 2) += (aPointList[i](0) - fXc)*(aPointList[i](2) - fZc);
			mA(1, 2) += (aPointList[i](1) - fYc)*(aPointList[i](2) - fZc);
		}
		mA(1, 0) = mA(0, 1);
		mA(2, 0) = mA(0, 2);
		mA(2, 1) = mA(1, 2);

		// for storing eigenvalues and eigenvectors
		VectorXd vEigenValues;
		MatrixXd vEigenVectors;

		// calculates all eigenvalues and corresponding eigenvectors of matrix mA
		mA.getEigenVectors(vEigenVectors, vEigenValues);

		// test Eigenvectors

		MatrixXd m1, m2, m3;

		// eigenvector that corresponds to smallest eigenvalue is normalized normal vector of fitted plane
		int iMax;
		if (vEigenValues(0) >= vEigenValues(1) && vEigenValues(0) >= vEigenValues(2))
			iMax = 0;
		else if (vEigenValues(1) > vEigenValues(0) && vEigenValues(1) >= vEigenValues(2))
			iMax = 1;
		else
			iMax = 2;

		// Geradengleichung: a*x + b*y + c*z +d = 0;
		// Berechnung des Normalenvektors
		vLineNorm(0) = vEigenVectors(0, iMax);
		vLineNorm(1) = vEigenVectors(1, iMax);
		vLineNorm(2) = vEigenVectors(2, iMax);
		vLineNorm = VEC_Norm(vLineNorm);

		vLinePoint(0) = fXc;
		vLinePoint(1) = fYc;
		vLinePoint(2) = fZc;
	}


} //End of the namespace TRTK 

#endif 

