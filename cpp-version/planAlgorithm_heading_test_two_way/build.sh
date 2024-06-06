#!/bin/bash

function all()
{
    prefix=$TARGET_TMPROOTFS_DIR

    [ -z "${BUILD_OUTPUT_PATH}" ] && export BUILD_OUTPUT_PATH="$(pwd)"
    # real build
    echo "-- BOARD_TYPE: ${BOARD_TYPE}"
    echo "-- SUB_BOARD: ${SUB_BOARD}"
    cmake -B${BUILD_OUTPUT_PATH}/build -H./ -DCMAKE_INSTALL_PREFIX=${BUILD_OUTPUT_PATH}/output \
        -DCROSS_COMPILE=$CROSS_COMPILE -DTARGET_TMPROOTFS_DIR=$TARGET_TMPROOTFS_DIR            \
        -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_BUILD_TYPE=$TARGET_MODE -DSRC_HBRE_DIR=$SRC_HBRE_DIR \
        -DBOARD_TYPE=$BOARD_TYPE -DTARGET_PREROOTFS_DIR=$TARGET_PREROOTFS_DIR \
        -DBUILD_ASAN_TYPE=$ENABLE_ASAN
    make -C ${BUILD_OUTPUT_PATH}/build -j$N || {
        echo "make failed"
        exit 1
    }
    make -C ${BUILD_OUTPUT_PATH}/build install
    ${CROSS_COMPILE}strip ${BUILD_OUTPUT_PATH}/output/lib/*

    mkdir -p $prefix/include/spihal
    cpfiles "${BUILD_OUTPUT_PATH}/output/include/*" "$prefix/include/spihal/"
    # libcutils
    cpfiles "${TARGET_PREROOTFS_DIR}/lib/libcutils.so" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/output/lib/*" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/output/examples/spi_service" "$prefix/bin/"
    cpfiles "${BUILD_OUTPUT_PATH}/output/config/*.json" "$prefix/etc/"
}

function all_32() 
{
    all
}

function clean()
{
    rm ${BUILD_OUTPUT_PATH}/ -rf
}

# include
. $INCLUDE_FUNCS

#dependon liblog

cd $(dirname $0)

buildopt $1


