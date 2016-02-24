
cd ../../libs/33_libfontconfig
tar xfvz fontconfig-2.10.2.tar.gz
cd fontconfig-2.10.2


echo Compile...

./configure --host=x86_64-w64-mingw32 --enable-libxml2 --disable-docs --enable-static --enable-shared  --prefix="$PREFIX"
make clean
make 2>&1 | tee ../../../z_Install/win64_build/logs/33_libfontconfig-make.log
make install 2>&1 | tee ../../../z_Install/win64_build/logs/33_libfontconfig-makeinstall.log


cd ..
rm -rf fontconfig-2.10.2
