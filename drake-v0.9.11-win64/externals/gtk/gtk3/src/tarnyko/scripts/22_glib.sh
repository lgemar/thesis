
cd ../../libs/22_glib
tar xfvJ glib-2.34.3.tar.xz
cd glib-2.34.3


echo To avoid using pkg-config...

# export LIBFFI_CFLAGS=-I"$PREFIX/lib/libffi-3.0.12/include"
# export LIBFFI_LIBS=-lffi

echo To avoid an annoying UI warning

cd ..
patch -p0 < gdbusaddress-mingw64.patch
cd glib-2.34.3

echo Warning, the main script should have taken care of putting required native
echo utils like glib-gennarshal in an accessible path. Otherwise, it will fail.

echo Save current compiler flags for later restoration

CFLAGS_SAVE="$CFLAGS"
CPPFLAGS_SAVE="$CPPFLAGS"

echo Add compiler flag needed to fix guint32 alignment detection at configure time

export CFLAGS="$CFLAGS -DSTDC_HEADERS"
export CPPFLAGS="$CPPFLAGS -DSTDC_HEADERS"

echo Add compiler flag needed to fix architecture detection at configure time

export CFLAGS="$CFLAGS -march=k8"
export CPPFLAGS="$CPPFLAGS -march=k8"

echo Add compiler flag needed to fix build problems

export CFLAGS="$CFLAGS -mms-bitfields -mthreads"
export CPPFLAGS="$CPPFLAGS -mms-bitfields -mthreads"

echo Patching a test because MinGW64 does not wrap the kill function...

cd ..
patch -p0 < gdbusproxyc-kill-win64.patch
cd glib-2.34.3

echo Compile...

./configure --host=x86_64-w64-mingw32 --with-pcre=internal --disable-static --enable-shared  --prefix="$PREFIX"
make clean
make 2>&1 | tee ../../../z_Install/win64_build/logs/22_glib-make.log
make install 2>&1 | tee ../../../z_Install/win64_build/logs/22_glib-makeinstall.log

echo Restore original compiler flags

export CFLAGS="$CFLAGS_SAVE"
export CPPFLAGS="$CPPFLAGS_SAVE"
unset CFLAGS_SAVE
unset CPPFLAGS_SAVE


cd ..
rm -rf glib-2.34.3
