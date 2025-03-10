#!/bin/bash

set -ex

if [ "$(uname)" != "Linux" ]; then
    echo "INFO: Skipping Yocto build on non-Linux platform."
    exit 0
fi

yocto_release=scarthgap

echo "- Yocto - Consume Mosquitto from Conan -"

rm -rf poky

echo "INFO: Cloning Yocto layers"
git clone --branch ${yocto_release} --depth 1 https://git.yoctoproject.org/poky.git poky
git clone --branch ${yocto_release} --depth 1 https://github.com/openembedded/meta-openembedded.git poky/meta-openembedded
git clone --branch conan2/${yocto_release} https://github.com/conan-io/meta-conan.git poky/meta-conan

mkdir -p poky/meta-conan/recipes-example
cp conan-mosquitto_2.0.18.bb poky/meta-conan/recipes-example/conan-mosquitto_2.0.18.bb

cd poky/
source oe-init-build-env

if [ -n "${CONAN_YOCTO_BUILD_MOSQUITTO}" ]; then

    echo "INFO: Adding layers"
    bitbake-layers add-layer ../meta-openembedded/meta-oe
    bitbake-layers add-layer ../meta-openembedded/meta-python
    bitbake-layers add-layer ../meta-conan

    echo "INFO: Present layers"
    bitbake-layers show-layers


    echo 'IMAGE_INSTALL:append = " conan-mosquitto"' >> conf/local.conf

    echo "INFO: Fetching mosquitto"
    bitbake -v -c fetch conan-mosquitto

    echo "INFO: Building mosquitto"
    bitbake -v -c configure conan-mosquitto
    bitbake -v -c compile conan-mosquitto
    bitbake -v -c package conan-mosquitto    
else:
    echo "INFO: Skipping mosquitto build due large time. Set the environment variable CONAN_YOCTO_BUILD_MOSQUITTO to build mosquitto."
fi

if [ -n "${CONAN_YOCTO_BUILD_IMAGE}" ]; then
    echo "INFO: Building core-image-minimal"
    bitbake -v core-image-minimal
else:
    echo "INFO: Skipping image build due large time. Set the environment variable CONAN_YOCTO_BUILD_IMAGE to build image."
fi
