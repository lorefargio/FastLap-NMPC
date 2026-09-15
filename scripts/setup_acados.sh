#!/usr/bin/env bash
# ==============================================================================
# setup_acados.sh
# Automated installer for acados, C libraries, t_renderer, and acados_template
# To be executed inside your container environment.
# ==============================================================================

set -e

ACADOS_INSTALL_DIR=${1:-"/opt/acados"}
VENV_DIR=${2:-"$HOME/.venv_mpc"}

echo "====================================================="
echo " Setting up acados for ETDV MPC"
echo " Target acados directory : ${ACADOS_INSTALL_DIR}"
echo " Target Python venv     : ${VENV_DIR}"
echo "====================================================="

# 1. Ensure required system build tools
echo "[1/5] Checking required build dependencies..."
if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -y && sudo apt-get install -y \
        cmake git build-essential python3-dev python3-pip python3-venv wget libopenblas-dev
fi

# 2. Clone or update acados
echo "[2/5] Fetching acados source repository..."
if [ ! -d "${ACADOS_INSTALL_DIR}" ]; then
    echo "Cloning acados to ${ACADOS_INSTALL_DIR}..."
    sudo git clone https://github.com/acados/acados.git "${ACADOS_INSTALL_DIR}"
    sudo chown -R $(whoami):$(id -gn) "${ACADOS_INSTALL_DIR}" 2>/dev/null || true
else
    echo "Directory ${ACADOS_INSTALL_DIR} already exists."
fi

cd "${ACADOS_INSTALL_DIR}"
git submodule update --recursive --init

# 3. Build acados C libraries with CMake
echo "[3/5] Building acados C libraries with qpOASES and HPIPM..."
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON \
      -DACADOS_WITH_OPENMP=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DACADOS_INSTALL_DIR="${ACADOS_INSTALL_DIR}" ..
make -j$(nproc)
make install

# 4. Download t_renderer (Tera code generator binary)
echo "[4/5] Downloading t_renderer binary..."
OS_TYPE=$(uname -s)
ARCH_TYPE=$(uname -m)

if [ "${OS_TYPE}" = "Linux" ]; then
    if [ "${ARCH_TYPE}" = "aarch64" ] || [ "${ARCH_TYPE}" = "arm64" ]; then
        T_RENDERER_URL="https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux"
    else
        T_RENDERER_URL="https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux"
    fi
elif [ "${OS_TYPE}" = "Darwin" ]; then
    T_RENDERER_URL="https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-osx"
fi

mkdir -p "${ACADOS_INSTALL_DIR}/bin"
if [ ! -f "${ACADOS_INSTALL_DIR}/bin/t_renderer" ]; then
    echo "Downloading ${T_RENDERER_URL}..."
    wget -O "${ACADOS_INSTALL_DIR}/bin/t_renderer" "${T_RENDERER_URL}"
    chmod +x "${ACADOS_INSTALL_DIR}/bin/t_renderer"
fi

# 5. Set up Python Virtual Environment & acados_template
echo "[5/5] Setting up Python virtual environment and acados_template..."
if [ ! -d "${VENV_DIR}" ]; then
    python3 -m venv "${VENV_DIR}"
fi

source "${VENV_DIR}/bin/activate"
pip install --upgrade pip setuptools wheel
pip install numpy scipy casadi matplotlib

# Install acados_template in editable mode
pip install -e "${ACADOS_INSTALL_DIR}/interfaces/acados_template"

echo ""
echo "====================================================="
echo " acados installation completed successfully!"
echo "====================================================="
echo "Add the following environment variables to your ~/.bashrc:"
echo "  export ACADOS_SOURCE_DIR=\"${ACADOS_INSTALL_DIR}\""
echo "  export LD_LIBRARY_PATH=\"${ACADOS_INSTALL_DIR}/lib:\$LD_LIBRARY_PATH\""
echo "  export PATH=\"${ACADOS_INSTALL_DIR}/bin:\$PATH\""
echo "  source \"${VENV_DIR}/bin/activate\""
echo "====================================================="
