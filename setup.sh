# Font styles
HEADER="\e[1;4m"
RESET="\e[0m"

# Install rosdeps for all climb-sdk packages
echo
echo -e "${HEADER}Checking rosdeps...${RESET}"
cd "$(dirname "$0")"
rosdep install --from-paths . --ignore-src -r -y
ROSDEP=$?

# Create the external packages folder if not present
mkdir -p "external" &&
cd external &&
touch "COLCON_IGNORE" &&
touch "AMENT_IGNORE" ||
{ echo "Failed to create external packages folder"; exit 1; }

# Install PIQP from source if not present
echo
echo -e "${HEADER}Checking PIQP...${RESET}"
cmake --find-package -DNAME=piqp -DCOMPILER_ID=GNU -DLANGUAGE=C -DMODE=EXIST
PIQP=$?
rm -rf CMakeFiles
if [ ! $PIQP -eq 0 ]; then
    ([ -d "piqp" ] || git clone https://github.com/PREDICT-EPFL/piqp.git) &&
    cd piqp &&
    mkdir -p build &&
    cd build &&
    cmake .. -DCMAKE_CXX_FLAGS="-march=native" -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF &&
    cmake --build . --config Release &&
    cmake --install . --config Release &&
    ldconfig
    PIQP=$?
fi

# Summary
SUCCESS="\e[32mSUCCESS\e[0m"
FAIL="\e[31mFAIL\e[0m"
echo
echo -e "${HEADER}Setup complete${RESET}"
echo -e "rosdeps: $([ $ROSDEP -eq 0 ] && echo ${SUCCESS} || echo ${FAIL})"
echo -e "PIQP: $([ $PIQP -eq 0 ] && echo ${SUCCESS} || echo ${FAIL})"
echo
