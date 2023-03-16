# Script to flash device with files from build folder
# Author: German Velardez     (germanvelardez16@gmail.com)
#
#
echo  "making somethings ... \n\t\t"
make

#file= $(ls ./build | grep .hex)

echo  "\n\n"
#read -p "introduce the name of the file .hex what you want to flash: " file

echo "flash the device..."
openocd -f interface/jlink.cfg -f target/stm32f1x.cfg    -c "program build/$(ls ./build | grep .hex) verify reset"


# Leer memoria
#read_memory 0xaddress numerodebits(8,16,32) cantidadaleer
#Escribir memoria
# write_memory 0xaddress numerodebits(8,16,32) {datos1,dato2}
