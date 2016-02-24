
cd ../../libs/44_gdk-pixbuf
tar xfvJ gdk-pixbuf-2.26.5.tar.xz
cd gdk-pixbuf-2.26.5


echo Compile...

./configure --host=x86_64-w64-mingw32 --with-included-loaders --enable-static --enable-shared  --prefix="$PREFIX"
make clean
make 2>&1 | tee ../../../z_Install/win64_build/logs/44_gdk-pixbuf-make.log
make install 2>&1 | tee ../../../z_Install/win64_build/logs/44_gdk-pixbuf-makeinstall.log


cd ..
rm -rf gdk-pixbuf-2.26.5
