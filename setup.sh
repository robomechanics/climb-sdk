echo
echo "Installing rosdeps:"
echo "================================="
cd "$(dirname "$0")"
rosdep install --from-paths .. --ignore-src -r -y

echo
echo "Installing Osqp-Eigen:"
echo "================================="

if ! [ -d "external" ]; then
    mkdir "external"
fi
cd "external"
touch COLCON_IGNORE

if [ -d "osqp-eigen" ]; then
    echo "#Osqp-Eigen already present"
else
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX:PATH="$(pwd)/../install" ../
    make
    make install
fi

echo
echo "Setup complete"
echo