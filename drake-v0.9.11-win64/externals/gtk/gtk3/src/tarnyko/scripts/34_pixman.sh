
cd ../../libs/34_pixman
tar xfvz pixman-0.26.0.tar.gz
cd pixman-0.26.0


echo Compile...

./configure --host=x86_64-w64-mingw32 --disable-sse2 --enable-static --enable-shared  --prefix="$PREFIX"
make clean
make 2>&1 | tee ../../../z_Install/win64_build/logs/34_pixman-make.log
make install 2>&1 | tee ../../../z_Install/win64_build/logs/34_pixman-makeinstall.log


cd ..
rm -rf pixman-0.26.0
