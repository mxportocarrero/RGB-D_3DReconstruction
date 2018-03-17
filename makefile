FLAGS = -lGL -lglut -lGLEW -lGLU -std=c++11 -w `pkg-config opencv --cflags --libs`

visualization = volumeintegrator.o tsdf.o
geometry = pointcloud.o odometry.o addfunctions.o
objects = image.o dataset.o camera.o loadshader.o main.o

%.o: %.cpp $(DEPS)
	g++ $(FLAGS) -c -o $@ $<

all: $(objects) $(geometry) $(visualization)
	g++ $(objects) $(geometry) $(visualization) -o app $(FLAGS)
clean:
	rm -f *.o app

