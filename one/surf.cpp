#include "surf.h"
#include "extra.h"
using namespace std;

#define PI 3.14159265

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }
	for(unsigned i = 0; i < 360; i++){
		Matrix3f rotationMatrix = Matrix3f( cos((i)*PI/180), 0.0f, sin((i)*PI/180),
											0.0f, 1.0f, 0.0f,
											-sin((i)*PI/180), 0.0f, cos((i)*PI/180));
		for(unsigned j = 0; j < profile.size(); j++){
			surface.VV.push_back(rotationMatrix*profile[j].V);
			surface.VN.push_back(rotationMatrix*(-profile[j].N));
			if(j > 0){
				if(i == 0){
					Tup3u a = Tup3u(((359) * profile.size()) + (j - 1), ((359)*profile.size())+j, j);
					Tup3u b = Tup3u(a[0], a[2], a[2] - 1);
					surface.VF.push_back(a);	
					surface.VF.push_back(b);	
				}else{
					Tup3u a = Tup3u(((i-1) * profile.size()) + (j - 1), ((i-1)*profile.size())+j, (i * profile.size()) + j);
					Tup3u b = Tup3u(a[0], a[2], a[2] - 1);
					surface.VF.push_back(a);	
					surface.VF.push_back(b);	
				}
			}
		}
	
	}

    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }
	Matrix3f rotationMatrix = Matrix3f( 1.0f, 0.0f, 0.0f,
										0.0f, cos(PI/2), -sin(PI/2),
										0.0f, sin(PI/2), cos(PI/2));
	for(unsigned i = 0; i < sweep.size(); i++){
		Matrix4f transformationMatrix = Matrix4f( sweep[i].N[0], sweep[i].B[0], sweep[i].T[0], sweep[i].V[0],
												sweep[i].N[1], sweep[i].B[1], sweep[i].T[1], sweep[i].V[1],
												sweep[i].N[2], sweep[i].B[2], sweep[i].T[2], sweep[i].V[2],
												0.0f, 0.0f, 0.0f, 1.0f);
		for(unsigned j = 0; j<profile.size(); j++){
			Matrix4f homogeneousMatrix = Matrix4f( -profile[j].N[0], profile[j].B[0], profile[j].T[0], profile[j].V[0],
												-profile[j].N[1], profile[j].B[1], profile[j].T[1], profile[j].V[1],
												-profile[j].N[2], profile[j].B[2], profile[j].T[2], profile[j].V[2],
												0.0f, 0.0f, 0.0f, 1.0f);
			Matrix4f resultingMatrix = transformationMatrix * homogeneousMatrix;
			Vector3f b = resultingMatrix.getCol(0).xyz();
			surface.VV.push_back(resultingMatrix.getCol(3).xyz());
			surface.VN.push_back(b);
			if(j > 0){
				if(i == 0){
					Tup3u c = Tup3u(((sweep.size()-1) * profile.size()) + (j - 1), ((sweep.size()-1)*profile.size())+j, j);
					Tup3u d = Tup3u(c[0], c[2], c[2] - 1);
					surface.VF.push_back(c);	
					surface.VF.push_back(d);	
				}else{
					Tup3u a = Tup3u(((i-1) * profile.size()) + (j - 1), ((i-1)*profile.size())+j, (i * profile.size()) + j);
					Tup3u b = Tup3u(a[0], a[2], a[2] - 1);
					surface.VF.push_back(a);	
					surface.VF.push_back(b);	
				}
			}
		}
	}
    // TODO: Here you should build the surface.  See surf.h for details.

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
