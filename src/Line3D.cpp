#include "TRTK/Line3D.hpp"


namespace TRTK
{

	/////////////////////////////////////////////////////////////////////////////////////	 
	Line3D::Line3D()
	{
		point = Vector3d(0, 0, 0);
		direction = Vector3d(0, 0, 1);
	}

	/** \tparam T scalar type
	*
	* \param [in] point on the line
	* \param [in] direction of the line
	*/

	Line3D::Line3D(const Vector3d &point, const Vector3d &direction)
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

	Line3D::Line3D(const Line3D  &line)
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

	bool Line3D::operator==(const Line3D & line2) const
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

	bool Line3D::operator!=(const Line3D & line1) const
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
	* \return Line3D
	*/

	Line3D  Line3D::operator*(const double & scalar) const
	{
		if (scalar == 0){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		return Line3D(point, (Vector3d(direction.x() * scalar, direction.y() * scalar, direction.z() * scalar)));

	}




	/** \tparam T scalar type
	*
	* \param  \c *this directions components will be divied component-wise by [in] scalar
	*
	* \return Line3D
	*/

	Line3D  Line3D::operator/(const double & scalar) const
	{
		if (scalar == 0){
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}
		return Line3D(point, (Vector3d(direction.x() / scalar, direction.y() / scalar, direction.z() / scalar)));

	}

	/** \tparam T scalar type
	*
	*
	*
	* \return true if the direction is normalized
	*
	*/

	bool Line3D::isNormalized()const{
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

	Vector3d Line3D::normalize() const{
		return direction * (1 / direction.norm());
	}


	// Test if the vectors are parallel 

	/** \tparam T scalar type
	*
	* \param [in] line2 line to be compared with \c *this
	*
	* \return true if vectors are parallel
	*/

	bool Line3D::isParallel(const Line3D &line2) const
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

	bool  Line3D::isPointOnTheLine(const Vector3d & point2) const
	{
		if (point2.size() < 3){
			ErrorObj error("The point hasn´t appropriate dimension \n");
			error.setClassName("Line3D");
			error.setFunctionName("isPointOnTheLine");
			throw error;
		}
		else{
			Vector3d m_point = point2 - point;
			Vector3d  diff_point = Vector3d(m_point.x() / direction.x(), m_point.y() / direction.y(), m_point.z() / direction.z());

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

	double Line3D::getAngleBetweenLines(const Line3D & line2) const
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
			Vector3d sum = line2.direction.cwiseProduct(direction);
			double coordinSumme = sum.x() + sum.y() + sum.z();



			return acos(coordinSumme / (direction.norm()*line2.direction.norm()));

		}


	}



	/** \tparam T scalar type
	*
	* \param The angle between [in] Vector2  and \c *this will be calculated
	*
	* \return the angle between the line and the vector
	*/

	double Line3D::getAngleBetweenVectorAndLine(const Vector3d & vector2) const
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

	bool Line3D::isIntersecting(const Line3D & line2) const
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

	double  Line3D::getMinDistance(const Line3D & line2)const{
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
		Vector3d vTmp2, vTmp;

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
			Vector3d diff = point - line2.point;
			vLambda = mMatrix.inverse() *mTemp.transpose() *  diff;
			Vector3d vpointOnLine1 = point + vLambda[0] * direction;
			Vector3d vPointOnLine2 = line2.point + vLambda[1] * line2.direction;

			return (vpointOnLine1 - vPointOnLine2).norm();
		}
	}
	//////////////////////////////////////////////////////////////////////////////
	/// Schnittpunkt von Ebene und Gerade bestimmen
	/// Übergeben werden:
	///	von der Ebene der normierte Normalenvektor (Normaleneinheitsvektor) und ein Punkt
	///	und von der Geraden der normierte Richtungsvektor und ein Punkt	
	/// \param		vPlaneNorm ist der Normalenvektor der Ebene, vPlanePoint ist ein Punkt auf der Ebene
	///				vLineDirection ist der Richtungsvektor der Geraden, vLinePoint ist ein Punkt auf der Geraden
	///	\return		Schnittpunkt der Ebene mit der Geraden
	///	\author		<a href="mailto:fuente@hia.rwth-aachen.de">Matias de la Fuente</a>
	/// \date		2001-2007
	///	\note		MathUtils::BIB 3.4.4.5
	///////////////////////////////////////////////////////////////////////////////

	Vector3d   Line3D::intersectionPoint(const Vector3d &_point) const
	{
		Vector3d  vIntersection;
		Vector3d vPlaneNorm = direction.unitOrthogonal();

		Vector3d vLineDirection = direction;
		Vector3d vPlanePoint = _point;
		Vector3d vLinePoint = point;

		// Ebenengleichung in Hessescher-Normalenform 
		//		(vIntersection - vPlanePoint) * vPlaneNorm = 0
		//
		// Geradengleichung
		//		vIntersection = vLinePoint + dLambda * vLineNorm
		//
		// Ergebnis
		//		vIntersection


		// gucken, dass Line nicht parallel zu Ebene liegt
		// Skalarprodukt zweier Vektoren ist Null, wenn die beiden Vektoren senkrecht aufeinander stehen,
		// d.h. wenn Normalenvektor der Ebene und Richtungsvektor der Geraden senkrecht sind, d.h. wenn
		// Ebene und Gerade parallel sind
		if (vPlaneNorm.dot(vLineDirection) != 0)
		{
			// dLambda erhält man durch Einsetzen der Geradengleichung in die Ebenengleichung
			double dLambda = (vPlanePoint.dot(vPlaneNorm) - vLinePoint.dot(vPlaneNorm)) / (vLineDirection.dot(vPlaneNorm));
			vIntersection = (vLinePoint + dLambda *vLineDirection);
			return vIntersection;
		}
		else
		{
			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}

	}
	/** \tparam T scalar type
	*
	* \param [in] line2 is the second line
	*
	* \return the point in which two lines are intersect
	*/

	/*
	*
	*
	*
	* ///////////////////////////////////////////////////////////////////////////////
	/// Lot eines Punktes auf eine Gerade.
	/// \param		vPoint Punkt, von dem aus das Lot auf die Gerade gefällt werden soll.
	///				vLinePoint Aufpunkt der Geraden und vLineDirection ist ihr Richtungsvektor
	///	\return		Lotfußpunkt
	///	\author		<a href="mailto:fuente@hia.rwth-aachen.de">Matias de la Fuente</a>
	/// \date		2001-2007
	///	\note		MathUtils::BIB 3.4.4.11
	///////////////////////////////////////////////////////////////////////////////
	CVector MathUtils::ProjectionPoint2Line(const CVector &vPoint, const CVector &vLinePoint, const CVector &vLineDirection)
	{
	return MathUtils::Intersection(vLineDirection, vPoint, vLineDirection, vLinePoint);
	}

	*/

	Vector3d   Line3D::intersectionLines(const Line3D  &line2) const
	{

		Vector3d vPlaneNorm = line2.direction;
		Vector3d vPlanePoint = line2.point;
		Vector3d vLineDirection = direction;
		Vector3d vLinePoint = point;

		Vector3d vIntersection;

		// Ebenengleichung in Hessescher-Normalenform
		//		(vIntersection - vPlanePoint) * vPlaneNorm = 0
		//
		// Geradengleichung
		//		vIntersection = vLinePoint + dLambda * vLineNorm
		//
		// Ergebnis
		//		vIntersection

		// gucken, dass Line nicht parallel zu Ebene liegt
		// Skalarprodukt zweier Vektoren ist Null, wenn die beiden Vektoren senkrecht aufeinander stehen,
		// d.h. wenn Normalenvektor der Ebene und Richtungsvektor der Geraden senkrecht sind, d.h. wenn
		// Ebene und Gerade parallel sind

		if (vPlaneNorm.dot(vLineDirection) != 0)
		{
			// dLambda erhält man durch Einsetzen der Geradengleichung in die Ebenengleichung
			double dLambda = (vPlanePoint.dot(vPlaneNorm) - vLinePoint.dot(vPlaneNorm)) / (vLineDirection.dot(vPlaneNorm));
			vIntersection = (vLinePoint + dLambda * vLineDirection);
			return vIntersection;
		}
		else{
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("intersectionLines");
			throw error;
		}


	}




	/** \tparam T scalar type
	*
	*
	* \return direction value of the line
	*/

	Vector3d  Line3D::getDirection() const
	{
		return direction;
	}

	/** \tparam T scalar type
	*
	*
	* \return a point of the line
	*
	*/

	Vector3d  Line3D::getPoint() const
	{
		return point;
	}


	/** \tparam T scalar type
	*
	* \param [in] direction2 is a new direction value instead of \c *this
	*
	*/

	void Line3D::setDirection(const Vector3d &direction2)
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

	void Line3D::setDirection(const double  &x, const  double &y, const double &z)
	{
		if (Vector3d(x, y, z).norm() == 0){
			ErrorObj error("Directons norm is null \n");
			error.setClassName("Line3D");
			error.setFunctionName("Construct");
			throw error;
		}

		direction = Vector3d(x, y, z);
	}


	//Set both values : point and direction

	/** \tparam T scalar type
	*
	* \param [in] point1 and [in] point2 are the new origin point of the line \c *this
	*
	*/

	void Line3D::set(const Vector3d &point1, const Vector3d &direction2)
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

	void Line3D::setPoint(const double &x, const double &y, const double &z)
	{
		point = Vector3d(x, y, z);
		direction = direction - point;
	}


	/** \tparam T scalar type
	*
	* \param [in] point2 is the new origin point of the line \c *this
	*
	*/

	void Line3D::setPoint(const Vector3d &point2)
	{
		point = point2;
		direction = point2 - point;
	}


	/** \tparam T scalar type
	*
	*
	* \return the orthogonal vector of the line
	*/

	Line3D  Line3D::getOrthogonal() const
	{
		Vector3d ret = direction.unitOrthogonal();

		if (ret.norm() == 0){

			ErrorObj error;
			error.setClassName("Line3D");
			error.setFunctionName(__FUNCTION__);
			throw error;
		}

		return Line3D(point, ret);
	}

	///////////////////////////////////////////////////////////////////////////////
	/// Lot eines Punktes auf eine Gerade.
	/// \param		vPoint Punkt, von dem aus das Lot auf die Gerade gefällt werden soll.
	///	\return		Lotfußpunkt
	///	\author		<a href="mailto:fuente@hia.rwth-aachen.de">Matias de la Fuente</a>
	/// \date		2001-2007
	///	\note		MathUnits::BIB 3.4.4.11
	///////////////////////////////////////////////////////////////////////////////

	Vector3d Line3D::getProjectionPoint2Line(const Vector3d &vPoint) const
	{/*CVector MathUtils::ProjectionPoint2Line(const CVector &vPoint, const CVector &vLinePoint, const CVector &vLineDirection)
	 {
	 return MathUtils::Intersection(vLineDirection, vPoint, vLineDirection, vLinePoint);
	 }*/
		Line3D  v_line(vPoint, direction);
		return intersectionLines(v_line);

	}
	/** \tparam T scalar type
	*
	* \param the minimal distance between [in] point2  and  \c *this will be calculated
	*
	* \return the minimal distance between the point and the line
	*/

	double Line3D::distancePoint2Line(const Vector3d &point2) const
	{
		// Zuerst eine Ebene bilden, mit Normalenvektor=Geradenrichtung und durch den vPoint
		// dann diese Gerade mit der Ebene schneiden
		// und anschliessen den Abstand von dem Schnittpunkt zu dem
		// ubergebenen Punkt berechnen
		if (isPointOnTheLine(point2)){
			return 0;
		}
		else{
			Line3D m_line(point2, direction);
			//Vector3d  vIntersection( intersectionLineAndPlane(m_line));


			double dLambda = (m_line.point.dot(m_line.direction.normalized()) - point.dot(m_line.direction.normalized())) / (direction.dot(m_line.direction.normalized()));
			Vector3d vIntersection = (point + dLambda * direction);
			return  (vIntersection - point2).norm();
		}

	}



	/** \tparam T scalar type
	*
	* \param equidistant point will be calculated between [in] line2  and \c *this
	*
	* \return  the equidistant point between two lines
	*/
	Vector3d Line3D::getMinDistancePoint(const Line3D  &line2) const
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
		Vector3d vTmp = direction.normalized().cross(line2.direction.normalized());
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
		Vector3d point_diff = (line2.point - point);

		//		m_temp(point_diff.x(), point_diff.y(), point_diff.z());
		Eigen::Vector3d vLambda = mEquations.inverse() * point_diff;

		// Der gesuchte Punkt liegt dann jetzt an der Stelle vLine2Point + vLambda(1) * vLine2Norm
		vTmp = point + vLambda.x() * direction.normalized() + (vLambda.z() * vTmp) / 2.0;

		// Den Punkt auf Line1, der den minimalen Abstand zu Line 2 hat in vPointOnLine1 zur�ckgeben
		//	vPointOnLine1 = vTmp;

		// R�ckgabewert ist der minimale Abstand der beiden Gerade
		return vTmp;
	}


	//void Line3D ::ausgleichsgerade(const	 <Vector3d> aPointList, const	 Vector3d &vLineNorm, const Vector3d &vLinePoint)
	//	{
	//// �berpr�fe, ob genug Punkte �bergeben werden
	//if (aPointList.size() < 3) {
	//	ErrorObj error;
	//	error.setClassName("Line3D");
	//	error.setFunctionName(__FUNCTION__);
	//	throw error;

	//}
	//// �berpr�fen, ob genug (verschiedene) Punkte �bergeben worden sind
	//unsigned int iCounter = 0;
	//for (unsigned int i = 0; i < aPointList.size() - 1; i++) {
	//	if (aPointList[i].x() == aPointList[i + 1].x() && aPointList[i].y() == aPointList[i + 1].y() && aPointList[i].z() == aPointList[i + 1].z()) {
	//		iCounter++;
	//		if (aPointList.size() - iCounter < 3) {

	//			ErrorObj error;
	//			error.setClassName("Line3D");
	//			error.setFunctionName(__FUNCTION__);
	//			throw error;


	//		}
	//	}
	//}

	//Eigen::MatrixXd mA;
	//mA.SetNull();

	//// mean point
	//double fXc = 0;
	//double fYc = 0;
	//double fZc = 0;
	//for (unsigned int i = 0; i < aPointList.size(); i++)
	//{
	//	fXc += aPointList[i].x();
	//	fYc += aPointList[i].y();
	//	fZc += aPointList[i].z();
	//}
	//// Mittelwert �ber alle Punkte bilden, dieser Punkt sollt dann auf der Ebene liegen.
	//fXc = fXc / aPointList.size();
	//fYc = fYc / aPointList.size();
	//fZc = fZc / aPointList.size();

	//// matrix mA is 3x3 symmetrical matrix 
	//// with sum of squared distances between points and mean point on main diagonal
	//// and sum of cross distances on off-diagonal
	//for (unsigned int i = 0; i < aPointList.size(); i++)
	//{
	//	mA(0, 0) += pow(aPointList[i](0) - fXc, 2);
	//	mA(1, 1) += pow(aPointList[i](1) - fYc, 2);
	//	mA(2, 2) += pow(aPointList[i](2) - fZc, 2);

	//	mA(0, 1) += (aPointList[i](0) - fXc)*(aPointList[i](1) - fYc);
	//	mA(0, 2) += (aPointList[i](0) - fXc)*(aPointList[i](2) - fZc);
	//	mA(1, 2) += (aPointList[i](1) - fYc)*(aPointList[i](2) - fZc);
	//}
	//mA(1, 0) = mA(0, 1);
	//mA(2, 0) = mA(0, 2);
	//mA(2, 1) = mA(1, 2);

	//// for storing eigenvalues and eigenvectors
	//VectorXd vEigenValues;
	//MatrixXd vEigenVectors;

	//// calculates all eigenvalues and corresponding eigenvectors of matrix mA
	//mA.getEigenVectors(vEigenVectors, vEigenValues);

	//// test Eigenvectors

	//MatrixXd m1, m2, m3;

	//// eigenvector that corresponds to smallest eigenvalue is normalized normal vector of fitted plane
	//int iMax;
	//if (vEigenValues(0) >= vEigenValues(1) && vEigenValues(0) >= vEigenValues(2))
	//	iMax = 0;
	//else if (vEigenValues(1) > vEigenValues(0) && vEigenValues(1) >= vEigenValues(2))
	//	iMax = 1;
	//else
	//	iMax = 2;

	//// Geradengleichung: a*x + b*y + c*z +d = 0;
	//// Berechnung des Normalenvektors
	//vLineNorm(0) = vEigenVectors(0, iMax);
	//vLineNorm(1) = vEigenVectors(1, iMax);
	//vLineNorm(2) = vEigenVectors(2, iMax);
	//vLineNorm = VEC_Norm(vLineNorm);

	//vLinePoint(0) = fXc;
	//vLinePoint(1) = fYc;
	//vLinePoint(2) = fZc;
	//}



} // namespace TRTK
