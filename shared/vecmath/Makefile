CXXFLAGS = -Wall -Wextra -pedantic -fPIC

objects =  Matrix2f.o Matrix3f.o Matrix4f.o Quat4f.o Vector2f.o Vector3f.o Vector4f.o
output = ./output

ver = 1.1
major = 1

vecmath: $(objects) headers
	$(CXX) -shared -Wl,-soname,libvecmath.so.$(major) -o $(output)/libvecmath.so.$(ver) $(objects)
	cd $(output)
	ln -sf libvecmath.so.$(ver) $(output)/libvecmath.so
	ln -sf libvecmath.so.$(ver) $(output)/libvecmath.so.$(major)

headers:
	mkdir -p $(output)/vecmath
	cp *.h   $(output)/vecmath

Matrix2f.o: Matrix2f.h Vector2f.h
Matrix3f.o: Matrix2f.h Matrix3f.h Quat4f.h Vector3f.h
Matrix4f.o: Matrix2f.h Matrix3f.h Matrix4f.h Quat4f.h Vector3f.h Vector4f.h
Quat4f.o: Quat4f.h Vector3f.h Vector4f.h
Vector2f.o: Vector2f.h Vector3f.h
Vector3f.o: Vector2f.h Vector3f.h
Vector4f.o: Vector2f.h Vector3f.h Vector4f.h

#This Makefile will compile and link the shared library vecmath to the output directory
#The individual makefiles for each assignment are located on its respectives directories
