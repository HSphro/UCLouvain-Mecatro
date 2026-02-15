#!/bin/bash
# Build script for DetectEnemy C++

set -e

echo "=========================================="
echo "DetectEnemy C++ - Build Script"
echo "=========================================="

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}[*] Working directory: $SCRIPT_DIR${NC}"

# Check for cmake
if ! command -v cmake &> /dev/null; then
    echo "[!] CMake not found. Installing..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get install -y cmake
    elif command -v brew &> /dev/null; then
        brew install cmake
    else
        echo "[ERROR] Could not install cmake. Please install manually."
        exit 1
    fi
fi

# Check for OpenCV
if ! pkg-config --exists opencv4 && ! pkg-config --exists opencv; then
    echo "[!] OpenCV not found. Installing..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get install -y libopencv-dev
    elif command -v brew &> /dev/null; then
        brew install opencv
    else
        echo "[ERROR] Could not install OpenCV. Please install manually."
        exit 1
    fi
fi

# Create build directory
if [ ! -d "$SCRIPT_DIR/build" ]; then
    echo -e "${BLUE}[*] Creating build directory...${NC}"
    mkdir -p "$SCRIPT_DIR/build"
fi

# Navigate to build directory
cd "$SCRIPT_DIR/build"

echo -e "${BLUE}[*] Configuring with CMake...${NC}"
cmake ..

echo -e "${BLUE}[*] Building...${NC}"
make -j$(nproc)

# Success message
echo -e "${GREEN}[✓] Build successful!${NC}"
echo -e "${GREEN}[✓] Executable: $SCRIPT_DIR/build/detect_enemy${NC}"
echo ""
echo "To run the program:"
echo "  cd $SCRIPT_DIR/build"
echo "  ./detect_enemy"
echo ""
echo "For help:"
echo "  ./detect_enemy --help"
