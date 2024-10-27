#include "MatrixStack.h"

MatrixStack::MatrixStack()
{
	// Initialize the matrix stack with the identity matrix.
	this->m_matrices.push_back(Matrix4f(1.0f, 0.0f, 0.0f, 0.0f,
										  0.0f, 1.0f, 0.0f, 0.0f,
										  0.0f, 0.0f, 1.0f, 0.0f,
										  0.0f, 0.0f, 0.0f, 1.0f));
}

void MatrixStack::clear()
{
	// Revert to just containing the identity matrix.
	this->m_matrices.clear();	
	this->m_matrices.push_back(Matrix4f(1.0f, 0.0f, 0.0f, 0.0f,
										  0.0f, 1.0f, 0.0f, 0.0f,
										  0.0f, 0.0f, 1.0f, 0.0f,
										  0.0f, 0.0f, 0.0f, 1.0f));
}

Matrix4f MatrixStack::top()
{
	// Return the top of the stack
	return this->m_matrices.back();
}

void MatrixStack::push( const Matrix4f& m )
{
	// Push m onto the stack.
	// Your stack should have OpenGL semantics:
	// the new top should be the old top multiplied by m
	Matrix4f resulting_matrix = this->m_matrices.back() * m;
	this->m_matrices.push_back(resulting_matrix);

}

void MatrixStack::pop()
{
	// Remove the top element from the stack
	this->m_matrices.pop_back();
}
