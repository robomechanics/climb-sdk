# Font styles
HEADER="\e[1;4m"
SUCCESS="\e[32m"
FAIL="\e[31m"
RESET="\e[0m"

# Install rosdeps for all climb-sdk packages
echo
echo -e "${HEADER}Installing rosdeps...${RESET}"
cd "$(dirname "$0")"
rosdep install --from-paths . --ignore-src -r -y
ROSDEP=$?

# Create the external packages folder if not present
if ! [ -d "external" ]; then
    mkdir "external"
    touch "external/COLCON_IGNORE"
    touch "external/AMENT_IGNORE"
fi
cd external
if [ $? -ne 0 ]; then
    echo
    echo -e "${FAIL}Error: failed to create external folder${RESET}"
    exit 1
fi

# Build PIQP from source if not present
echo
echo -e "${HEADER}Installing PIQP...${RESET}"
if [ -d "piqp" ]; then
    PIQP=0
    echo "#PIQP already present"
else
    git clone https://github.com/PREDICT-EPFL/piqp.git &&
    cd piqp &&
    mkdir build &&
    cd build &&
    cmake .. -DCMAKE_CXX_FLAGS="-march=native" -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF &&
    cmake --build . --config Release &&
    cmake --install . --config Release
    PIQP=$?
    echo
    if [ $PIQP -eq 0 ]; then
        echo -e "#PIQP installed successfully"
    else
        echo -e "#PIQP installation failed"
    fi
fi

# Summary
echo
echo -e "${HEADER}Setup complete${RESET}"
if [ $ROSDEP -eq 0 ]; then
    echo -e "rosdeps: ${SUCCESS}SUCCESS${RESET}"
else
    echo -e "rosdeps: ${FAIL}FAIL${RESET}"
fi
if [ $PIQP -eq 0 ]; then
    echo -e "PIQP: ${SUCCESS}SUCCESS${RESET}"
else
    echo -e "PIQP: ${FAIL}FAIL${RESET}"
fi
echo
