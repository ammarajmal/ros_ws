CXX ?= g++
INCLUDE = -I/usr/include/opencv4 -I../../include
LIBS = -L/usr/lib/x86_64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_highgui -lMVSDK

all: main
main:
	$(CXX) $(INCLUDE) -o main main.cpp $(LIBS)
clean:
	rm -f *.o 
	rm -f main 
