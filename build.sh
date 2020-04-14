if [ -e build/ ]; then
	echo 'delete build/ directory'
	rm -rf build/
fi

if [ -e /home/rtemsdev/development/rtems4.11/arm-rtems4.11/nuc970series/ ]; then
	echo 'delete nuc970series/ directory'
	rm -rf /home/rtemsdev/development/rtems4.11/arm-rtems4.11/nuc970series/
fi
./bootstrap -c
./bootstrap -p
./bootstrap

mkdir build
cd build
../configure --target=arm-rtems4.11 --enable-cxx --enable-networking --enable-posix --enable-rtemsbsp=nuc970series --prefix=/home/rtemsdev/development/rtems4.11
make
make install
