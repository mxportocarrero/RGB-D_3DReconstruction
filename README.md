
# RGB-D - 3D Reconstruction

Proyecto Final para Computación Gráfica - Programa de Maestría en Ciencia de la Computación - UCSP


# Instalación y Compilación

Este Proyecto funciona en base a OpenCV y OpenGL

Para Compilar la Libreria Eigen

descargar el repo en Github

git clone https://github.com/eigenteam/eigen-git-mirror.git

dentro crear una carpeta /build

cd build
cmake source_dir/
make install



# Instalacion de Opencv3.4.1con python3 en anaconda O.o Compilando desde source
tener en cuenta que contaba con cuda version 9.0 y su cuDNN 7.0
con los drivers adecuados ver 384

todo corriendo con normalidad

tambien ya tenia instalado anaconda 4.5.4

Con esto compile el opencv 3.4.1 (Active el support de Qt para el high-ui de Qt)
Debería detectar los paths correctos a las versiones 2 y 3 de python e instalarlas normalmente
sobretodo si estamos usando anaconda

cmake -DWITH_QT=ON \
    -DCMAKE_INSTALL_PREFIX=$(python -c "import sys; print(sys.prefix)") \
    -DPYTHON3_EXECUTABLE=$(which python) \
    -DPYTHON3_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -DPYTHON3_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") ../

or for python3 en ubuntu 18.04
cmake -DWITH_QT=ON \
    -DCUDA_NVCC_FLAGS=--expt-relaxed-constexpr \
    -DCMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \
    -DPYTHON3_EXECUTABLE=$(which python3) \
    -DPYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -DPYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") ../

make -j4
make install
make uninstall // para desinstalar


# Tipicos errores

Package opencv was not found in the pkg-config search path

	Solucion: buscar el archivo opencv.pc y exportarlo a las variables del sistema
	usualmente

	export PKG_CONFIG_PATH=/usr/lib/pkgconfig
	export PKG_CONFIG_PATH=~/anaconda3/lib/pkgconfig // Con anaconda3

libopencv_features2d.so.3.4: cannot open shared object file: No such file or directory

	Error por que no encuentra la libreria correcta para correr el programa.

	Solucion: se debe exportar la dirección correcta a la libreria que pida

	export LD_LIBRARY_PATH=/usr/local/lib/
	export LD_LIBRARY_PATH=~/anaconda3/lib/   // en caso de anaconda3

Tenia un error en el renderizado. Esto se generaba por no inicializar correctamente las matrices de transformacion del Modelo

    Model = glm::rotate(mat4(),180.0f,vec3(1,0,0)); //* glm::scale(mat4(),vec3(scale,scale,scale)); // Pasar la matriz(mat4()) sin parametros me daba error.
    // Desconfiguraba la ubicacion de los objetos a renderizar
    // Esto derepente fue generado debido a un cambio de version de la librería glm.
    // Específicamente debido a que cambie de maquina
    Model = glm::rotate(mat4(1.0f),180.0f,vec3(1,0,0)); //* glm::scale(mat4(),vec3(scale,scale,scale));

# Aditional Notes

    En la PC que tengo actualmente en el labo
    Intel® Core™ i7-7700 CPU, 16GB RAM, GeForce GTX 1070(8GB)
    he logrado correr mi algoritmo hasta con 200 frames.. (300->500) usando alrededor de 70% de capacidad

