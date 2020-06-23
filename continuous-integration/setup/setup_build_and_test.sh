#!/bin/bash

echo Start ["$(basename $0)"]

UBUNTU_VERSION="$(lsb_release -rs)" || exit $?

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit $?
apt-yes dist-upgrade || exit $?

apt-yes install \
    clinfo \
    wget \
    python3-pip \
    python-catkin-tools \
    unzip \
    || exit $?

function install_opencl_cpu_runtime {

    # Install driver prerequisites
    apt-yes install libnuma-dev lsb-core || exit $?

    TMP_DIR=$(mktemp --tmpdir --directory zivid-setup-opencl-cpu-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    wget -q https://www.dropbox.com/s/h0txd04aqfluglq/l_opencl_p_18.1.0.015.tgz || exit $?
    tar -xf l_opencl_p_18.1.0.015.tgz || exit $?
    cd l_opencl_*/ || exit $?

    cat > installer_config.cfg <<EOF
# See silent.cfg in the .tgz for description of the options.
ACCEPT_EULA=accept
# 'yes' below is required because the installer officially supports only Ubuntu 16.04. However, it will
# work fine on Ubuntu 18.04 as well.
CONTINUE_WITH_OPTIONAL_ERROR=yes
PSET_INSTALL_DIR=/opt/intel
CONTINUE_WITH_INSTALLDIR_OVERWRITE=yes
COMPONENTS=DEFAULTS
PSET_MODE=install
INTEL_SW_IMPROVEMENT_PROGRAM_CONSENT=no
SIGNING_ENABLED=yes
EOF
    echo "Running Intel OpenCL driver installer."
    echo "Note: Installer will warn about 'Unsupported operating system' if not run on Ubuntu 16.04."
    echo "This warning can be ignored."
    echo
    ./install.sh --silent installer_config.cfg || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

install_opencl_cpu_runtime || exit $?

echo "clinfo:"
clinfo || exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    echo "Downloading Zivid debian package $1"
    wget -q "$@" || exit $?
    echo "Installing Zivid debian package $1"
    apt-yes install --fix-broken ./*deb || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

echo "Installing $CI_TEST_COMPILER"
if [[ "$CI_TEST_COMPILER" == "g++-7" ]] ||
   [[ "$CI_TEST_COMPILER" == "g++-8" ]] ||
   [[ "$CI_TEST_COMPILER" == "g++-9" ]]; then
    apt-yes install software-properties-common || exit $?
    add-apt-repository -y ppa:ubuntu-toolchain-r/test || exit $?
    apt-yes update || exit $?
    apt-yes install $CI_TEST_COMPILER || exit $?
elif [[ "$CI_TEST_COMPILER" == "clang++-7" ]]; then
    apt-yes install clang-7 || exit $?
else
    echo "Unhandled CI_TEST_COMPILER $CI_TEST_COMPILER"
    exit 1
fi

echo "Install Zivid and Telicam debian packages"

ZIVID_RELEASE_DIR="https://www.zivid.com/hubfs/softwarefiles/releases/$CI_TEST_ZIVID_VERSION"
ZIVID_TELICAM_SDK_DEB="zivid-telicam-driver_3.0.1.1-2_amd64.deb"

if [[ "$UBUNTU_VERSION" == "16.04" ]]; then

    install_www_deb "$ZIVID_RELEASE_DIR/u16/${ZIVID_TELICAM_SDK_DEB}" || exit $?
    install_www_deb "$ZIVID_RELEASE_DIR/u16/zivid_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?

elif [[ "$UBUNTU_VERSION" == "18.04" ]]; then

    install_www_deb "$ZIVID_RELEASE_DIR/u18/${ZIVID_TELICAM_SDK_DEB}" || exit $?
    install_www_deb "$ZIVID_RELEASE_DIR/u18/zivid_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?

else
    echo "Unhandled OS $UBUNTU_VERSION"
    exit 1
fi

echo Success! ["$(basename $0)"]
