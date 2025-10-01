#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen\Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)


ASplineVec3::ASplineVec3() : mInterpolator(new ABernsteinInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    if (mInterpolator) delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

	if (mInterpolator) { delete mInterpolator; }
    switch (type)
    {
	case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
	case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
	case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
	case LINEAR_EULER: mInterpolator = new AEulerLinearInterpolatorVec3(); break;
	case CUBIC_EULER: mInterpolator = new AEulerCubicInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
	// debug
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints(false);
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
		computeControlPoints(false);
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

int ASplineVec3::insertKey(double time, const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(time, value, updateCurve);
		return 0;
	}

	for (int i = 0; i < mKeys.size(); ++i)
	{
		assert(time != mKeys[i].first);
		if (time < mKeys[i].first)
		{
			mKeys.insert(mKeys.begin() + i, Key(time, value));
			if (updateCurve)
			{
				computeControlPoints();
				cacheCurve();
			}
			return i;
		}
	}

	// Append at the end of the curve
	appendKey(time, value, updateCurve);
	return mKeys.size() - 1;
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}
vec3 ASplineVec3::getKey(int keyID) const
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID) const
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size() + 2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys.size() == 0 ? 0 : mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

double ASplineVec3::getKeyTime(int keyID) const
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].first;
}

vec3 ASplineVec3::getValue(double t) const
{
    if (mCachedCurve.size() == 0 || mKeys.size() == 0) return vec3();
	if (t < mKeys[0].first)
		return mCachedCurve[0];
	else
		t -= mKeys[0].first;

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    double frac = (t - rawi*dt) / dt;

	int i = mLooping? rawi % mCachedCurve.size() : std::min<int>(rawi, mCachedCurve.size() - 1);
	int inext = mLooping ? (i + 1) % mCachedCurve.size() : std::min<int>(i + 1, mCachedCurve.size() - 1);

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints(bool updateEndPoints)
{
	if (mKeys.size() >= 2 && updateEndPoints)
	{
		int totalPoints = mKeys.size();

		//If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
		//They lie on the tangent of the first and last interpolation points.
		vec3 tmp = mKeys[0].second - mKeys[1].second;
		double n = tmp.Length();
		mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

		tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
		n = tmp.Length();
		mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
	}
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

vec3* ASplineVec3::getCachedCurveData()
{
	return mCachedCurve.data();
}

vec3 * ASplineVec3::getControlPointsData()
{
	return mCtrlPoints.data();
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0;

	curve.clear();

	int numSegments = keys.size() - 1;
	for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
			// u = 0.0 when t = keys[segment-1].first  
			// u = 1.0 when t = keys[segment].first
			double u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);

            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments - 1, u);
		curve.push_back(val);
	}
}


// Interpolate p0 and p1 so that t = 0 returns p0 and t = 1 returns p1
vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
	curveValue = (1 - u) * key0 + u * key1;
	return curveValue;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	double w = 1.0 - u;
	curveValue = w * w * w * b0 + 3 * w * w * u * b1 + 3 * w * u * u * b2 + u * u * u * b3;

	return curveValue;
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	// Step2: Compute the interpolated value f(u) point using deCsteljau alogithm
	// Level 1
	vec3 b01 = b0 + u * (b1 - b0);
	vec3 b12 = b1 + u * (b2 - b1);
	vec3 b23 = b2 + u * (b3 - b2);

	// Level 2
	vec3 b012 = b01 + u * (b12 - b01);
	vec3 b123 = b12 + u * (b23 - b12);

	// Level 3
	curveValue = b012 + u * (b123 - b012);

	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations
	Eigen::MatrixXd G(4, 3);
	G << b0[0], b0[1], b0[2],
		 b1[0], b1[1], b1[2],
		 b2[0], b2[1], b2[2],
		 b3[0], b3[1], b3[2];

	// Bezier basis matrix
	Eigen::MatrixXd M(4, 4);
	M << -1,  3, -3, 1,
		  3, -6,  3, 0,
		 -3,  3,  0, 0,
		  1,  0,  0, 0;

	// U row vector [u^3, u^2, u, 1]
	Eigen::RowVector4d U(u * u * u, u * u, u, 1.0);

	// Multiply (1x4) * (4x4) * (4x3) = (1x3)
	Eigen::RowVector3d result = U * M * G;
	curveValue =  vec3(result(0), result(1), result(2));

	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 p0;
	vec3 p1;
	vec3 q0; // slope at p0
	vec3 q1; // slope at p1
	vec3 curveValue(0, 0, 0);

	// TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial  
	p0 = keys[segment].second;
	p1 = keys[segment + 1].second;
	q0 = ctrlPoints[segment];
	q1 = ctrlPoints[segment + 1];

	double u2 = u * u;
	double u3 = u2 * u;

	// Hermite basis
	double h00 = 2 * u3 - 3 * u2 + 1;
	double h10 = u3 - 2 * u2 + u;
	double h01 = -2 * u3 + 3 * u2;
	double h11 = u3 - u2;

	curveValue = h00 * p0 + h10 * q0 + h01 * p1 + h11 * q1;

	return curveValue;
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);

	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	// Step 2: compute the n nonzero Bspline Basis functions N given j
	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t

	return curveValue;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    for (int i = 1; i < keys.size(); i++)
    {
        vec3 b0, b1, b2, b3;
		// TODO: compute b0, b1, b2, b3
		vec3 p_im1, p_i, p_ip1, p_ip2;
		p_i = keys[i - 1].second;
		p_ip1 = keys[i].second;

		// handle p_im1 previous neighbor
		if (i == 1) {
			p_im1 = p_i - (p_ip1 - p_i);
		}
		else {
			p_im1 = keys[i - 2].second;
		}

		// handle p_ip2 next neighbor
		if (i == static_cast<int>(keys.size()) - 1) {
			p_ip2 = p_ip1 + (p_ip1 - p_i);
		}
		else {
			p_ip2 = keys[i + 1].second;
		}

		b0 = p_i;
		b1 = p_i + (p_ip1 - p_im1) / 6.0;
		b2 = p_ip1 - (p_ip2 - p_i) / 6.0;
		b3 = p_ip1;

        ctrlPoints.push_back(b0);
        ctrlPoints.push_back(b1);
        ctrlPoints.push_back(b2);
        ctrlPoints.push_back(b3);
    }
}

void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	ctrlPoints.resize(keys.size(), vec3(0, 0, 0));
	if (keys.size() <= 1) return;

	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	const int n = static_cast<int>(keys.size());

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
	A(0, 0) = 2; A(0, 1) = 1;
	A(n - 1, n - 2) = 1; A(n - 1, n - 1) = 2;
	for (int i = 1; i < n - 1; i++) {
		A(i, i - 1) = 1;
		A(i, i) = 4;
		A(i, i + 1) = 1;
	}

	// Step 2: Initialize D
	Eigen::MatrixXd D(n, 3);
	auto putRow = [&](int r, const vec3& v) {
		// write vec3 components into the row
		D(r, 0) = v.n[0]; D(r, 1) = v.n[1]; D(r, 2) = v.n[2];
	};

	putRow(0, 3.0 * (keys[1].second - keys[0].second));
	putRow(n - 1, 3.0 * (keys[n - 1].second - keys[n - 2].second));
	for (int i = 1; i < n - 1; ++i) {
		putRow(i, 3.0 * (keys[i + 1].second - keys[i - 1].second));
	}

	// Step 3: Solve AC=D for C
	Eigen::MatrixXd C = A.inverse() * D;

	// Step 4: Save control points in ctrlPoints
	for (int i = 0; i < n; i++) {
		ctrlPoints[i] = vec3(C(i, 0), C(i, 1), C(i, 2));
	}

	// Control Points: [p0_prime, p1_prime, p2_prime, ..., pm_prime]
	// The size of control points should be the same as the size of the keys
	// Use operator[] to set elements in ctrlPoints by indices

	// Hint: Do not use push_back() to insert control points here because the vector has been resized

}

void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
	ctrlPoints.resize(keys.size() + 2, vec3(0, 0, 0));
    if (keys.size() <= 1) return;

	// TODO:
	// Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C

	// 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)

	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	// Step 4: Solve AC=D for C 

	// Step 5: save control points in ctrlPoints

	// Hint: Do not use push_back() to insert control points here because the vector has been resized
}

vec3 CalculateShortestPath(const vec3& a, const vec3& b)
{
	vec3 B2(0, 0, 0);
	for (int i = 0; i < 3; ++i) {
		double diff = b[i] - a[i];

		// Normalize difference to [-180,180]
		diff = std::remainder(diff, 360.0);

		B2[i] = a[i] + diff;
	}
	return B2;
}

vec3 AEulerLinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, 
	int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO:
	// Linear interpolate between key0 and key1
	// You should convert the angles to find the shortest path for interpolation
	key0 = CalculateShortestPath(vec3(0, 0, 0), key0);
	key1 = CalculateShortestPath(key0, key1);

	curveValue = key0 + u * (key1 - key0);

	return curveValue;
}

vec3 AEulerCubicInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	// You should convert the angles to find the shortest path for interpolation
	curveValue = pow((1 - t), 3) * b0 + 3 * t * pow((1 - t), 2) * b1 + 3 * pow(t, 2) * (1 - t) * b2 + pow(t, 3) * b3;

	return curveValue;
}

void AEulerCubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys, 
	std::vector<vec3>& ctrlPoints, vec3 & startPoint, vec3 & endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	// Hint: One naive way is to first convert the keys such that the differences of the x, y, z Euler angles 
	//		 between every two adjacent keys are less than 180 degrees respectively
	int n = keys.size();

	for (int i = 1; i < keys.size(); i++)
	{
		vec3 b0, b1, b2, b3;
		vec3 p0, p1, p2, p3;
		double t0, t1, t2, t3;

		// TODO: compute b0, b1, b2, b3
		// Grab neighbors
		p1 = keys[i - 1].second;
		p2 = keys[i].second;
		t1 = keys[i - 1].first;
		t2 = keys[i].first;

		// Handle boundaries
		if (i > 1) {
			p0 = keys[i - 2].second;
			t0 = keys[i - 2].first;
		}
		else {
			p0 = p1; // duplicate for left boundary
			t0 = t1;
		}

		if (i < n - 1) {
			p3 = keys[i + 1].second;
			t3 = keys[i + 1].first;
		}
		else {
			p3 = p2; // duplicate for right boundary
			t3 = t2;
		}

		// Recalculate with shortest path correction
		p0 = CalculateShortestPath(vec3(0, 0, 0), p0);
		p1 = CalculateShortestPath(p0, p1);
		p2 = CalculateShortestPath(p1, p2);
		p3 = CalculateShortestPath(p2, p3);

		// Compute slopes
		vec3 s1, s2;

		if (i == 1) { // left endpoint slope forward diff
			s1 = (p2 - p1) / (t2 - t1);
		}
		else {
			vec3 s_left = (p1 - p0) / (t1 - t0);
			vec3 s_right = (p2 - p1) / (t2 - t1);
			s1 = (s_left + s_right) / 2.0;
		}

		if (i == n - 1) { // right endpoint slope backward diff
			s2 = (p2 - p1) / (t2 - t1);
		}
		else {
			vec3 s_left = (p2 - p1) / (t2 - t1);
			vec3 s_right = (p3 - p2) / (t3 - t2);
			s2 = (s_left + s_right) / 2.0;
		}

		// Hermite to Bezier conversion
		b0 = p1;
		b3 = p2;
		b1 = b0 + s1 / 3.0;
		b2 = b3 - s2 / 3.0;

		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}
