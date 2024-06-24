g++ -c -fPIC c3bf.cpp -o c3bf.o
g++ -shared -Wl,-soname,c3bf.so -o c3bf.so  c3bf.o



