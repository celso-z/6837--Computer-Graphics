#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::lineToJoint(string line){

	Joint *j = new Joint();
	float translations[3];
	int position, parent_index;

	for(int i = 0; i < 3; i++){
		position = line.find(' ');
		translations[i] = std::stof(line.substr(0, position));
		line.erase(0, position + 1);
	}
	position = line.find(' ');
	parent_index = std::stoi(line.substr(0, position));

	j->transform = Matrix4f(1.0f,0.0f,0.0f, translations[0],
							0.0f, 1.0f, 0.0f, translations[1],
							0.0f, 0.0f, 1.0f, translations[2],
							0.0f, 0.0f, 0.0f, 1.0f);
	if(parent_index >= 0){
		Joint *parent = m_joints[parent_index];	
		parent->children.push_back(j);	
		j->currentJointToWorldTransform = parent->currentJointToWorldTransform * j->transform;
		j->bindWorldToJointTransform = j->transform * parent->currentJointToWorldTransform.inverse();
	}else if(parent_index == -1){
	
		//TODO: -1 this->m_rootJoint
		this->m_rootJoint = j;
		j->currentJointToWorldTransform = j->transform;
		j->bindWorldToJointTransform = j->transform.inverse();

	}
	m_joints.push_back(j);

}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
	std::fstream fs;
	string line;
	fs.open(filename, std::fstream::in);
	if(fs.fail()){
		cerr << "Skeleton not found!" << endl;
		exit(-1);	
	}
	if (fs.is_open()){
		int i = 0;
		while(getline(fs, line)){
			lineToJoint(line);
		}
	}
	
}

static void traverseJointHierarchy( Joint *root, MatrixStack stack){

	stack.push(root->transform);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(stack.top());

    // Load the custom transformation matrix
	glutSolidSphere(0.025f, 12, 12);
	glFlush();
	for(int i = 0; i < root->children.size(); i++){
		Joint *child = root->children[i];
		Vector4f homogeneousTranslation = child->transform.getCol(3);
		float distL = Vector3f(homogeneousTranslation[0], homogeneousTranslation[1], homogeneousTranslation[2]).abs();
		Matrix4f transform = Matrix4f(0.05f,0.0f,0.0f, 0.0f,
								0.0f, 0.05f, 0.0f, 0.0f,
								0.0f, 0.0f, distL, distL/2,
								0.0f, 0.0f, 0.0f, 1.0f);

		Vector3f rnd = Vector3f(0,0,1);
		Vector3f parentOffset = 1*Vector3f(homogeneousTranslation[0], homogeneousTranslation[1], homogeneousTranslation[2]);
		Vector3f z = parentOffset.normalized();
		Vector3f y = Vector3f::cross(z,rnd).normalized();
		Vector3f x = Vector3f::cross(y,z).normalized();
		Matrix4f coordReset = Matrix4f(x[0], y[0], z[0], 0.0f,
				       x[1], y[1], z[1], 0.0f,
				       x[2], y[2], z[2], 0.0f,
				       0.0f, 0.0f, 0.0f, 1.0f);
		stack.push(coordReset);

		stack.push(transform);
		glLoadMatrixf(stack.top());
		glutSolidCube(1.0f);
		stack.pop();
		stack.pop();
		traverseJointHierarchy(child, stack);	
	}
	stack.pop();
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
	
	Matrix4f initial_matrix = Matrix4f();
	glGetFloatv(GL_MODELVIEW_MATRIX, initial_matrix);
	m_matrixStack = MatrixStack();
	m_matrixStack.push(initial_matrix);
	traverseJointHierarchy( this->m_rootJoint, m_matrixStack);
	
	
							
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
}

