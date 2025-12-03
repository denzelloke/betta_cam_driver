# FFMPEG Hardware Acceleration

## Minimal Setup for Jetson - Current Working Configuration

* Use the repository [jocover/jetson-ffmpeg](https://github.com/jocover/jetson-ffmpeg) and follow the instructions in the README to build and install the library, create the FFmpeg patch, and build. This setup uses `nvmpi` for FFmpeg hardware acceleration.

* Use the following configuration flags instead of the ones given in the repository:

```bash
./configure --enable-nvmpi \
            --disable-everything \
            --enable-encoder=h264_nvmpi \
            --enable-decoder=h264_nvmpi \
            --enable-protocol=file \
            --enable-muxer=mp4 \
            --enable-demuxer=h264 \
            --enable-demuxer=mov \
            --enable-demuxer=mp4 \
            --enable-encoder=aac \
            --enable-decoder=aac \
            --enable-shared \
            --disable-static \
            --enable-decoder=mjpeg \
            --prefix=/home/aibx/jetson-ffmpeg/build/ffmpeg
```

* After configuring, run:

```bash
make
sudo make install
```

* Set up environment variables:

```bash
export PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg:$PATH
export LD_LIBRARY_PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg/lib:$LD_LIBRARY_PATH
```

* Add these "Above export lines" to your `.bashrc` for persistence:

---

## Extensive FFmpeg Setup Including CUDA

### NVCodec Setup

```bash
sudo rm -rf /usr/local/include/ffnvcodec/
sudo rm -f /usr/local/lib/pkgconfig/ffnvcodec.pc
cd ~
rm -rf nv-codec-headers
git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
cd nv-codec-headers
make
sudo make install
pkg-config --modversion ffnvcodec
pkg-config --cflags ffnvcodec
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH"
```

### LibAOM Setup

```bash
cmake --version
cd ~
git clone https://aomedia.googlesource.com/aom
cd aom
git checkout v3.3.0
cd build
cmake .. \
  -DENABLE_SHARED=1 \
  -DENABLE_TESTS=0 \
  -DENABLE_DOCS=0 \
  -DENABLE_EXAMPLES=0 \
  -DENABLE_PKGCONFIG=1 \
  -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
cd /usr/local/lib/pkgconfig
sudo ln -s aom.pc libaom.pc
ls -l libaom.pc
sudo ldconfig
pkg-config --modversion libaom
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH"
```

### FFmpeg Setup

```bash
mkdir ~/jetson-ffmpeg
cd jetson-ffmpeg/
mkdir build
cd build
git clone https://git.ffmpeg.org/ffmpeg.git ffmpeg
cd ffmpeg
make distclean
```

#### Configuration (Without NVMP Patch, Focusing on CUDA/NVENC)

```bash
PKG_CONFIG_PATH="/usr/local/lib/pkgconfig" ./configure \
--disable-everything \
--enable-gpl \
--enable-nonfree \
--enable-cuda-nvcc \
--enable-libnpp \
--enable-cuda \
--enable-cuvid \
--enable-nvenc \
--enable-shared \
--disable-static \
--enable-protocol=file \
--enable-decoder=h264,mjpeg \
--enable-demuxer=h264,mov,mp4 \
--enable-encoder=h264_nvenc \
--enable-muxer=mp4 \
--extra-cflags="-I/usr/local/include -I/usr/local/include/ffnvcodec -I/usr/local/cuda/include" \
--extra-ldflags="-L/usr/local/lib -L/usr/local/cuda/lib64" \
--prefix=/home/aibx/jetson-ffmpeg/build/ffmpeg
```

* After configuring, run:

```bash
make
sudo make install
```

* Set up environment variables:

```bash
export PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg:$PATH
export LD_LIBRARY_PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg/lib:$LD_LIBRARY_PATH
```
