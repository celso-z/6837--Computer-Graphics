#include "curve.h"
#include "extra.h"
#include <cmath>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

#define Polinomials(t) Vector4f(1.0f, (t), (t*t), (t*t*t))

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}

/*static Vector3f deCasteljau( vector< Vector3f >& controlPoints, float position){
	if(controlPoints.size() == 1){
		return controlPoints[0];
	}
	vector<Vector3f> novoControlPoints;

	for(unsigned i = 0; i < controlPoints.size() - 1; i++){
			novoControlPoints.push_back((1.0-position) * controlPoints[i] + position * controlPoints[i + 1]);
	}
	return getCurvePositionPoint(novoControlPoints, position);
}
*/

Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
	Matrix4f bezierMatrix = Matrix4f( 1.0f, -3.0f,  3.0f,-1.0f,
									  0.0f,  3.0f, -6.0f, 3.0f,
									  0.0f,  0.0f,  3.0f,-3.0f,
									  0.0f,  0.0f,  0.0f, 1.0f);
	Matrix4f bezierMatrixDerived = Matrix4f(-3.0f,  6.0f, -3.0f, 0.0f,
											 3.0f,-12.0f,  9.0f, 0.0f,
											 0.0f,  6.0f, -9.0f, 0.0f,
											 0.0f,  0.0f,  3.0f, 0.0f);


	Matrix4f controlPointsMatrix = Matrix4f(P[0][0],P[1][0],P[2][0],P[3][0],
											P[0][1],P[1][1],P[2][1],P[3][1],
											P[0][2],P[1][2],P[2][2],P[3][2],
											0.0f, 0.0f, 0.0f, 0.0f);
	Curve c;

    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }


    cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> [" << P[i][0] << "," << P[i][1] << "," << P[i][2] << "]" << endl;
    }

	for(unsigned i = 0; i < steps; i++){
		CurvePoint p;
		float t = (1.0f / steps) * i; 
		Vector4f curveAtPoint = controlPointsMatrix * bezierMatrix * Polinomials(t);
		Vector4f derivativeAtPoint = controlPointsMatrix * bezierMatrixDerived * Polinomials(t);
		derivativeAtPoint.normalize();
		p.V = Vector3f(curveAtPoint[0], curveAtPoint[1], curveAtPoint[2]);
		p.T = Vector3f(derivativeAtPoint[0], derivativeAtPoint[1], derivativeAtPoint[2]);
		Vector3f b;
		if(i == 0){
			b = p.T;
			b[2] += 1.0f;
		}
		else b = c[i - 1].B;
		p.N = Vector3f::cross(b, p.T).normalized();
		p.B = Vector3f::cross(p.T, p.N).normalized();
		c.push_back(p);
	}
	
    // Right now this will just return this empty curve.
    return c;
}


Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

	Matrix4f splineMatrix = Matrix4f((1.0f/6), (-3.0f/6), (3.0f/6), (-1.0f/6),
									(4.0f/6),  (0.0f),  (-1.0f),    (3.0f/6),
									(1.0f/6),  (3.0f/6), (3.0f/6), (-3.0f/6),
									(0.0f),    (0.0f),   (0.0f),    (1.0f/6));

	Matrix4f bezierMatrixInverse = Matrix4f( 1.0f, -3.0f,  3.0f,-1.0f,
									  0.0f,  3.0f, -6.0f, 3.0f,
									  0.0f,  0.0f,  3.0f,-3.0f,
									  0.0f,  0.0f,  0.0f, 1.0f).inverse();
	
	//G = G * Bspline * Bbezier (-1 inv) para cada conjunto de 4 pontos
	
	Curve completeSpline;
	for(unsigned i = 0; i < P.size() - 3; i++){
		vector< Vector3f > subCurve(P.begin() + i, P.begin() + i+4);	
		Matrix4f controlPointsMatrix = Matrix4f(subCurve[0][0],subCurve[1][0],subCurve[2][0],subCurve[3][0],
											subCurve[0][1],subCurve[1][1],subCurve[2][1],subCurve[3][1],
											subCurve[0][2],subCurve[1][2],subCurve[2][2],subCurve[3][2],
											0.0f, 0.0f, 0.0f, 0.0f);
		Matrix4f newControlPointsMatrix = controlPointsMatrix * splineMatrix * bezierMatrixInverse; //converted to the new matrix
		vector< Vector3f > bezierEquivalent;
		for(int j = 0; j < 4; j++){
			bezierEquivalent.push_back(Vector3f(newControlPointsMatrix(0,j),newControlPointsMatrix(1,j),newControlPointsMatrix(2,j)));
		}
		
		Curve subSpline = evalBezier(bezierEquivalent, steps);
		completeSpline.insert(completeSpline.end(), subSpline.begin(), subSpline.end());
	
	}

    // Return an empty curve right now.
    return completeSpline;
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

