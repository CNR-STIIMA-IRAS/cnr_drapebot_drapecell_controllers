# ATTENTION:
The official release of Mosquitto for Windows provides only the static library, to work properly this packege needs the mosquitto library compiled as dynamic library.
As a workaround to use this package in Windows it is necessary to compile mosquitto from source as .dll and change the CMakeLists.txt include_directories and target_link_directories paths according with the path where the mosquitto library was compiled.
Then it is necessary to copy mosquitto.dll in your_ws_path/devel/bin/