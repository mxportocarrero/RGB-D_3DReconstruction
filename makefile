FLAGS = -lGL -lglut -lGLEW -lGLU -std=c++11 -w `pkg-config opencv --cflags --libs`

visualization = volumeintegrator.cpp
geometry = pointcloud.cpp odometry.cpp addfunctions.cpp
objects = image.cpp dataset.cpp camera.cpp loadshader.cpp main.cpp

all: $(objects)
	g++ $(objects) $(geometry) $(visualization) -o app $(FLAGS)


clean:
	rm -f *.o app

