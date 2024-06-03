g++ -c -fPIC stl_hocbf.cpp -o stl_hocbf.o
g++ -shared -Wl,-soname,stl_hocbf.so -o stl_hocbf.so  stl_hocbf.o


