build=3
cppen=1
networken=0


remove_obj()
{
	if [ -e build/ ]; then
	echo 'delete build/ directory'
	rm -rf build/
	fi

	if [ -e /home/rtemsdev/development/rtems4.11/arm-rtems4.11/nuc970series/ ]; then
		echo 'delete nuc970series/ directory'
		rm -rf /home/rtemsdev/development/rtems4.11/arm-rtems4.11/nuc970series/
	fi
}

boot_strap()
{
	./bootstrap -c
	./bootstrap -p
	./bootstrap
}


while [ "$1" != "" ] ; do 
	case $1 in 
		fullbuild )
			build=1
			;;
		halfbuild )
			build=2
			;;
		easybuild )
			build=3
			;;
		cppon )
			cppen=1
			;;
		cppoff )
			cppen=0
			;;
		networkon )
			networken=1
			;;
		networkoff )
			networken=0
			;;
	esac
	shift
done




if [ "$build" = "1" ]; then
	remove_obj
	boot_strap
	mkdir build
	cd build
	if [ "$networken" = "1" ] ; then
		../configure --target=arm-rtems4.11 --enable-cxx --enable-networking --enable-posix --enable-rtemsbsp=nuc970series --prefix=/home/rtemsdev/development/rtems4.11
	else
		../configure --target=arm-rtems4.11 --enable-cxx --disable-networking --enable-posix --enable-rtemsbsp=nuc970series --prefix=/home/rtemsdev/development/rtems4.11
	fi
	make
	make install
elif [ "$build" = "2" ]; then
	remove_obj
	mkdir build
	cd build
	if [ "$networken" = "1" ] ; then
		../configure --target=arm-rtems4.11 --enable-cxx --enable-networking --enable-posix --enable-rtemsbsp=nuc970series --prefix=/home/rtemsdev/development/rtems4.11
	else
		../configure --target=arm-rtems4.11 --enable-cxx --disable-networking --enable-posix --enable-rtemsbsp=nuc970series --prefix=/home/rtemsdev/development/rtems4.11
	fi
	make
	make install
else
	cd build
	make
	make install
fi

