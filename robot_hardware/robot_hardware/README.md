# 智身小狗切换arm与x86架构
CMakeLists.txt
set(ARCH "aarch64") 
set(ARCH "x86_64")

mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

# 卸载
sudo make uninstall

# 宇树狗自测
cd build 
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
先让狗站立(遥控器L2+B)
然后解除锁定(遥控器 START)
./robot_test

