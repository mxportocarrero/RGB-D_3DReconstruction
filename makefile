FLAGS = -lGL -lglut -lGLEW -lGLU -std=c++11 -w `pkg-config opencv --cflags --libs`

objects = image.cpp dataset.cpp camera.cpp loadshader.cpp pointcloud.cpp main.cpp

all: $(objects)
	g++ $(objects) -o app $(FLAGS) && ./app


clean:
	rm -f *.o app

