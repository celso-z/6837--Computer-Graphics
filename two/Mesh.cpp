#include "Mesh.h"


using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
	fstream fs;

	fs.open(filename, fstream::in);
	if(fs.fail()){
		cerr << "Could not load Mesh, could not open file!" << endl;
		exit(-1);
	}

	char entry;
	while(fs >> entry){
		
		if(entry == 'v'){
			Vector3f v = Vector3f();
			fs >> v[0] >> v[1] >> v[2];
			this->bindVertices.push_back(v);
		}else if(entry == 'f'){
			unsigned t[3] = {0,0,0};
			fs >> t[0] >> t[1] >> t[2];
			this->faces.push_back(Tuple3u(t[0] -1, t[1] - 1, t[2] - 1));
		}
	
	}

	// make a copy of the bind vertices as the current vertices
	this->currentVertices = this->bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
	
	glBegin(GL_TRIANGLES);
	glDisable( GL_LIGHTING );
    for (unsigned i=0; i<this->faces.size(); i++)
    {
		Vector3f point1 = this->currentVertices[faces[i][0]];
		Vector3f point2 = this->currentVertices[faces[i][1]];
		Vector3f point3 = this->currentVertices[faces[i][2]];
		Vector3f u = point2 - point1;
		Vector3f v = point3 - point1;
		Vector3f normal =  Vector3f::cross(u, v).normalized();

        glNormal3f(normal[0], normal[1], normal[2]);
        glVertex3f(point1[0], point1[1], point1[2]);
        glVertex3f(point2[0], point2[1], point2[2]);
        glVertex3f(point3[0], point3[1], point3[2]);
    }
	glEnable( GL_LIGHTING );
    glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
}
