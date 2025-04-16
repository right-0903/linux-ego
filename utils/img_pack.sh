#!/bin/bash
# =============================================================================
# Filename: img_pack.sh
# Purpose: To be filled
# Usage: sudo ./img_pack.sh
# =============================================================================

# libarchive-tools for bsdtar, qemu-user-static for emulation
# parted for partion raw img, arch-install-scripts for arch-chroot, genfstab
apt update && apt install -y zstd curl libarchive-tools qemu-user-static parted arch-install-scripts

# handle binfmt_misc, https://access.redhat.com/solutions/1985633
if grep -q 'binfmt_misc' /proc/mounts; then
    echo "binfmt_misc mounted"
else
    mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
fi

echo 1 > /proc/sys/fs/binfmt_misc/status
echo ':qemu-aarch64:M::\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\xb7\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-aarch64-static:FP' > /proc/sys/fs/binfmt_misc/register


# first serval loop device are taken up
LOOP_DEV='/dev/loop7'

# container setup
dd of=archlinuxarm.img bs=1 seek=3G count=0
losetup ${LOOP_DEV} archlinuxarm.img
parted ${LOOP_DEV} --script mklabel gpt
parted ${LOOP_DEV} --script mkpart EFI fat32 1MiB 301MiB
parted ${LOOP_DEV} --script set 1 boot on
parted ${LOOP_DEV} --script mkpart ALARM ext4 301MiB 100%
mkfs.fat -F32 ${LOOP_DEV}p1
mkfs.ext4 ${LOOP_DEV}p2

# mount, extract rootfs and generate a mount table
CHROOT_DIR='alarm-chroot'
mkdir ${CHROOT_DIR}
mount ${LOOP_DEV}p2 ${CHROOT_DIR}
MIRROR_URL='http://fl.us.mirror.archlinuxarm.org'
curl "$MIRROR_URL/os/ArchLinuxARM-aarch64-latest.tar.gz" > alarm.tar.gz
bsdtar -xpf alarm.tar.gz -C ${CHROOT_DIR}
mkdir ${CHROOT_DIR}/boot/efi
mount ${LOOP_DEV}p1 ${CHROOT_DIR}/boot/efi

###### dirty insert ######

genfstab -U ${CHROOT_DIR} >> ${CHROOT_DIR}/etc/fstab

# github action runner has a swap, we don't need it.
sed -i '/^.*swap.*$/d' ${CHROOT_DIR}/etc/fstab

# many tutorials sugget this
cp /usr/bin/qemu-aarch64-static  ${CHROOT_DIR}/usr/bin/qemu-aarch64-static

# enable ParallelDownloads
sed -i 's/#ParallelDownloads = 5/ParallelDownloads = 4/' ${CHROOT_DIR}/etc/pacman.conf
echo "Server = $MIRROR_URL"'/$arch/$repo' >> ${CHROOT_DIR}/etc/pacman.d/mirrorlist

# add my repo to install kernel and firmware
echo '[nuvole-arch]' >> ${CHROOT_DIR}/etc/pacman.conf
echo "Server = https://github.com/right-0903/my_arch_auto_pack/releases/download/packages" >> ${CHROOT_DIR}/etc/pacman.conf

# set console font
cat << EOF >> ${CHROOT_DIR}/etc/vconsole.conf
KEYMAP=us
FONT=solar24x32
EOF

# disable all kinds of sleep for now
cat << EOF >> ${CHROOT_DIR}/etc/systemd/sleep.conf
AllowSuspend=no
AllowHibernation=no
AllowSuspendThenHibernate=no
AllowHybridSleep=no
EOF

# initialize the pacman keyring and populate the Arch Linux ARM package signing keys
# https://archlinuxarm.org/platforms/armv8/generic
arch-chroot ${CHROOT_DIR} sh -c 'pacman-key --init && pacman-key --populate archlinuxarm'

# trust my key for my repo
curl https://raw.githubusercontent.com/right-0903/my_arch_auto_pack/refs/heads/main/keys/CA909D46CD1890BE.asc -o ${CHROOT_DIR}/root/CA909D46CD1890BE.asc
arch-chroot ${CHROOT_DIR} sh -c 'pacman-key --add /root/CA909D46CD1890BE.asc && pacman-key --lsign-key CA909D46CD1890BE'

# make life easier
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Rn linux-aarch64 --noconfirm'

# update and install something needed, then clean immediately because of container space
arch-chroot ${CHROOT_DIR} sh -c 'pacman -Syu efibootmgr grub linux-firmware-qcom wireless-regdb --noconfirm && rm /var/cache/pacman/pkg/*'

arch-chroot ${CHROOT_DIR} sh -c 'pacman -S linux-gaokun3 linux-gaokun3-headers linux-firmware-gaokun3 --noconfirm'

# add iwd to for wifi configuration, someone reported this, btrfs-progs for people using btrfs
arch-chroot ${CHROOT_DIR} sh -c 'pacman -S iwd btrfs-progs --noconfirm'

# make a copy for this repo
mv ${CHROOT_DIR}/var/cache/pacman/pkg/*.pkg.tar.zst .

# use early KMS for debugging, this would give us log in the initramfs stage.
sed -i 's/^\(MODULES=(\)/\1\nsimpledrm\nphy-qcom-snps-femto-v2/' ${CHROOT_DIR}/etc/mkinitcpio-gaokun3.conf
arch-chroot ${CHROOT_DIR} sh -c 'mkinitcpio -P'

# install grub
arch-chroot ${CHROOT_DIR} sh -c "grub-install ${LOOP_DEV}p1"

# fix efi loading
arch-chroot ${CHROOT_DIR} sh -c 'mkdir /boot/efi/EFI/Boot && cp /boot/efi/EFI/arch/grubaa64.efi /boot/efi/EFI/Boot/bootaa64.efi'

# set kernel commandline parameters
sed -i 's/GRUB_CMDLINE_LINUX=""/GRUB_CMDLINE_LINUX="clk_ignore_unused pd_ignore_unused arm64.nopauth iommu.passthrough=0 iommu.strict=0 pcie_aspm.policy=powersupersave efi=noruntime modprobe.blacklist=msm"/' ${CHROOT_DIR}/etc/default/grub
sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="loglevel=3 quiet"/GRUB_CMDLINE_LINUX_DEFAULT="fbcon=rotate:1 loglevel=3"/' ${CHROOT_DIR}/etc/default/grub

# generate the grub config
arch-chroot ${CHROOT_DIR} 'update-grub'

# do clean
rm ${CHROOT_DIR}/usr/bin/qemu-aarch64-static ${CHROOT_DIR}/root/*

# umount
umount -R ${CHROOT_DIR} && losetup -d ${LOOP_DEV}

# compress, github release is limited to 2GB
xz archlinuxarm.img
