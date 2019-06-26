#!/bin/bash
#
# Configures a raspbian image to operate a vigibot
# Vigibot image is created from the official raspbian image.
#

set -e
set -u

DOWNLOAD_DIR="/tmp/raspbian"
RASPBIAN_IMG_URL="https://downloads.raspberrypi.org/raspbian_lite_latest"
RASPBIAN_ZIP=${DOWNLOAD_DIR}/raspbian_lite_latest.zip
RASPBIAN_IMG=
BOOT_PARTITION=
ROOT_PARTITION=
ROOT_DIR=$(mktemp -d)
BOOT_DIR=${ROOT_DIR}/boot
NO_DELETE=
NO_CLEAN=



#
#
function cleanup()
{
    [ "${NO_CLEAN}" != "" ] && exit 0

    umount_image

    echo "cleanup"

    if [ "${NO_DELETE}" == "" ] ; then
        if [ "${DOWNLOAD_DIR}" != "" -a -d ${DOWNLOAD_DIR} ] ; then
            rm -rf ${DOWNLOAD_DIR}
        fi
    fi
}


trap "cleanup" EXIT TERM INT

function install_tools()
{
    local dependencies="curl kpartx qemu-user-static binfmt-support"
    apt update
    apt install -y ${dependencies}
}

function get_image()
{
    mkdir -p ${DOWNLOAD_DIR}
    curl -L -o ${RASPBIAN_ZIP} ${RASPBIAN_IMG_URL}
    unzip ${RASPBIAN_ZIP} -d ${DOWNLOAD_DIR}
}


function mount_image()
{
    RASPBIAN_IMG=$(ls ${DOWNLOAD_DIR}/*.img)

    if [ "${RASPBIAN_IMG}" != "" ] ; then
        BOOT_PARTITION=$(kpartx -asv ${RASPBIAN_IMG} | grep "loop[0-9]p1" | sed -e 's/.*\(loop[0-9]p1\).*/\1/')
        ROOT_PARTITION=$(kpartx -asv ${RASPBIAN_IMG} | grep "loop[0-9]p2" | sed -e 's/.*\(loop[0-9]p2\).*/\1/')

        if [ "${BOOT_PARTITION}" != "" -a "${ROOT_PARTITION}" != "" ] ; then
            mount "/dev/mapper/${ROOT_PARTITION}" "${ROOT_DIR}"
            mount "/dev/mapper/${BOOT_PARTITION}" "${BOOT_DIR}"
        fi
    fi
}

# TODO check mounted partitions and device mapping
function umount_image()
{
    if [ "${BOOT_PARTITION}" != "" ] ; then
        umount "/dev/mapper/${BOOT_PARTITION}" || echo "failed to umount ${BOOT_PARTITION}"
    fi

    if [ "${ROOT_PARTITION}" != "" ] ; then
        umount "/dev/mapper/${ROOT_PARTITION}" || echo "failed to umount ${ROOT_PARTITION}"
    fi

    if [ "${RASPBIAN_IMG}" != "" ] ; then
        kpartx -d "${RASPBIAN_IMG}" || echo "failed to delete device map"
    fi
}

function update_image()
{
    if [ -d "${ROOT_DIR}/usr/bin" ] ; then
        cp /usr/bin/qemu-arm-static "${ROOT_DIR}/usr/bin"
        cp install.sh ${ROOT_DIR}/tmp
        chroot ${ROOT_DIR} /tmp/install.sh
    fi
}

function export_image()
{
    if [ -f ${RASPBIAN_IMG} ] ; then
        zip vigimage.zip ${RASPBIAN_IMG}
    fi
}

for i in $@ ; do
    case "${1}" in
    "--no-clean")
        NO_CLEAN=1
        shift
    ;;
    "--no-delete")
        NO_DELETE=1
        shift
    ;;
    "--all")
        get_image
        update_image
        shift
    ;;
    "--debug")
        ls /dev/mapper/*
        mount |grep loop
        exit 0
    ;;
    "--exec")
        ${2}
        exit 0
    ;;
    *)
    ;;
    esac
done


