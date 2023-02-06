## ¿Qué necesito?
Si prefieres usar directamente la línea de comandos para generar y compilar el proyecto, primero debes asegurarte que tienes:
Un compilador C++ que admita el estándar C++11.
Una herramienta para compilar proyectos en línea de comandos, en linux será make y en Windows puedes usar NMake.
La herramienta CMake.
Un editor de código, por ejemplo Gedit en Linux o Notepad++ en Windows.
Si el proyecto depende de una librería externa, como por ejemplo OpenCV, esta debe estar instalada en el sistema y accesible. Por ejemplo en Linux en /usr y en Windows en c:\OpenCV con la variable de entorno OpenCV apuntando dicha carpeta.

## Installing Build tools

```
sudo apt-get install cmake build-essential g++ pkg-config libopencv-dev
```


## Cómo configurar el proyecto
Si tienes lo necesario, los pasos a seguir para configurar y compilar el proyecto serían:
Entrar en la carpeta del proyecto.
CreSar una carpeta para compilar (normalmente la llamamos “build”).
Configura el proyecto (este paso se hace sólo una vez).
Ya puedes empezar el ciclo de codificar - compilar - codificar - ….
Por ejemplo en la línea de comandos de Linux estos pasos serían::

```
mkdir build
cd build
cmake ..
make
./show_img ../data/avion.png
```