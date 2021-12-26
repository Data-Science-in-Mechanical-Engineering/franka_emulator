#!/bin/sh

# Installing dependencies
apt-get install build-essential bc ca-certificates gnupg2 libssl-dev libelf-dev bison flex dwarves expect initramfs-tools systemd
if [ $? -ne 0 ]; then echo Failure: could not install dependencies; exit 1; fi

# Detecting version
if [ $# -eq 0 ]; then
	KERNELMAJOR=$(uname -r | cut -d '.' -f 1)
	KERNELMINOR=$(uname -r | cut -d '.' -f 2)
	KERNELPATCH=$(uname -r | cut -d '.' -f 3 | cut -d '-' -f 1)
	if [ "${KERNELMAJOR}" = "" ] \
	|| [ "${KERNELMINOR}" = "" ] \
	|| [ "${KERNELPATCH}" = "" ]; then echo Failure: invalid arguments; exit 1; fi
elif [ $# -eq 1 ]; then
	KERNELMAJOR=$(cat "$1" | cut -d '.' -f 1)
	KERNELMINOR=$(cat "$1" | cut -d '.' -f 2)
	KERNELPATCH=$(cat "$1" | cut -d '.' -f 3 | cut -d '-' -f 1)
else echo Failure: invalid arguments; exit 1; fi

# Downloading source
if [ -d linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH} ]; then
	rm -rf linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}
	if [ $? -ne 0 ]; then echo Failure: could not delete linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}; exit 1; fi
fi
if [ ! -f linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar ]; then
	if [ ${KERNELPATCH} -eq 0 ]; then
		wget https://kernel.org/pub/linux/kernel/v${KERNELMAJOR}.x/linux-${KERNELMAJOR}.${KERNELMINOR}.tar.xz -O linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar.xz
	else
		wget https://kernel.org/pub/linux/kernel/v${KERNELMAJOR}.x/linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar.xz -O linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar.xz
	fi
	if [ $? -ne 0 ]; then echo Failure: could download linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}; exit 1; fi
	xz -d -v linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar.xz
	if [ $? -ne 0 ]; then echo Failure: could not decompress linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar.xz; exit 1; fi	
fi
mkdir linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}
if [ $? -ne 0 ]; then echo Failure: could not create linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH} directory; exit 1; fi
tar xfv linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar -C linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}
if [ $? -ne 0 ]; then echo Failure: could not decompress linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}.tar; exit 1; fi
if [ $(find linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}/ -maxdepth 1 -type f | wc -l) -eq 0 ]; then mv linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}/*/* linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}/; fi

# Downloading patch
if [ ! -f patch-${KERNELMAJOR}.${KERNELMINOR}-index.html ]; then
	wget https://kernel.org/pub/linux/kernel/projects/rt/${KERNELMAJOR}.${KERNELMINOR}/older -O patch-${KERNELMAJOR}.${KERNELMINOR}-index.html
	if [ $? -ne 0 ]; then echo Failure: could not download patch-${KERNELMAJOR}.${KERNELMINOR} older patches list; exit 1; fi
fi
if [ ${KERNELPATCH} -eq 0 ]; then
	KERNELSUFFIX=$(cat patch-${KERNELMAJOR}.${KERNELMINOR}-index.html | grep href=\"patch-"${KERNELMAJOR}.${KERNELMINOR}-.*.patch.xz\"" -o | tail -1 | cut -d '"' -f 2 | cut -d '.' -f 1-2 | cut -d '-' -f 3)
else
	KERNELSUFFIX=$(cat patch-${KERNELMAJOR}.${KERNELMINOR}-index.html | grep href=\"patch-"${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-.*.patch.xz\"" -o | tail -1 | cut -d '"' -f 2 | cut -d '.' -f 1-3 | cut -d '-' -f 3)
fi

if [ ! -f patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch ]; then
	if [ ${KERNELPATCH} -eq 0 ]; then
		wget https://kernel.org/pub/linux/kernel/projects/rt/${KERNELMAJOR}.${KERNELMINOR}/older/patch-${KERNELMAJOR}.${KERNELMINOR}-${KERNELSUFFIX}.patch.xz -O patch-${KERNELMAJOR}.${KERNELMINOR}.0-${KERNELSUFFIX}.patch.xz
	else
		wget https://kernel.org/pub/linux/kernel/projects/rt/${KERNELMAJOR}.${KERNELMINOR}/older/patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch.xz -O patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch.xz
	fi
	if [ $? -ne 0 ]; then echo Failure: could not download patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch; exit 1; fi
	xz -d -v patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch.xz
	if [ $? -ne 0 ]; then echo Failure: could not decompress patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch.xz; exit 1; fi
fi

# Patching
cd linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}
patch -p1 < ../patch-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}.patch
if [ $? -ne 0 ]; then echo Failure: could not patch linux-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}; exit 1; fi
	
# Configuring
cat <<EOF > ../config.expect
set timeout -1
spawn make config
set send_slow { 1 1 }
expect {
	"*(PREEMPT_RT)*]*" { send -s -- "4\r"; set send_slow { 1 0.05 }; exp_continue }
	"*(SYSTEM_TRUSTED_KEYS)*]*" { send -s -- "SYSTEM_TRUSTED_KEYS_PLACEHOLDER.pem\r"; exp_continue }
	"*]*" { send -s -- "\r"; exp_continue }
	eof
}
EOF
expect ../config.expect
if [ $(cat .config | grep CONFIG_PREEMPT_RT=y | wc -l) -eq 0 ]; then echo Failure: could not configure preemption model; exit 1; fi
if [ $(cat .config | grep SYSTEM_TRUSTED_KEYS_PLACEHOLDER.pem | wc -l) -eq 0 ]; then echo Failure: could not configure additional X.509 keys for default system keyring; exit 1; fi
sed 's/SYSTEM_TRUSTED_KEYS_PLACEHOLDER.pem//g' .config -i

# Building
make -j$(nproc)
if [ $? -ne 0 ]; then echo Failure: could not compile kernel; exit 1; fi
make modules -j$(nproc)
if [ $? -ne 0 ]; then echo Failure: could not compile kernel modules; exit 1; fi
make bzImage -j$(nproc)
if [ $? -ne 0 ]; then echo Failure: could not compile kernel image; exit 1; fi

# Insalling
if [ $(blkid | grep EFI | grep '/dev/[a-z0-9]*' -o | wc -l) -ne 1 ]; then echo Failure: could not autodetect EFI device; exit 1; fi
EFIDEV=$(blkid | grep EFI | grep '/dev/[a-z0-9]*' -o)
if [ $(mount | grep ${EFIDEV} | grep 'on /[a-zA-z/]*' -o | grep '/[a-zA-z/]*' -o | wc -l) -ne 1 ]; then echo Failure: could not autodetect EFI directory; exit 1; fi
EFIDIR=$(mount | grep ${EFIDEV} | grep 'on /[a-zA-z/]*' -o | grep '/[a-zA-z/]*' -o)
if [ $(mount | grep 'on /[^a-z]' | grep '/dev/[a-z0-9]*' -o | wc -l) -ne 1 ]; then echo Failure: could not autodetect root device; exit 1; fi
ROOTDEV=$(mount | grep 'on /[^a-z]' | grep '/dev/[a-z0-9]*' -o)
if [ $(blkid | grep ${ROOTDEV} | grep '[^a-zA-Z]UUID=\"[a-zA-Z0-9-]*\"' -o | cut -d '"' -f 2 | wc -l) -ne 1 ]; then echo Failure: could not autodetect root UUID; exit 1; fi
ROOTUUID=$(blkid | grep ${ROOTDEV} | grep '[^a-zA-Z]UUID=\"[a-zA-Z0-9-]*\"' -o | cut -d '"' -f 2)
make modules_install
if [ $? -ne 0 ]; then echo Failure: could not install kernel modules; exit 1; fi
find /lib/modules/${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}/ -type f -name '*.ko' -exec echo Stripping {} \; -exec strip --strip-unneeded {} \;
echo Creating initramfs image
mkinitramfs -o initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX} ${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
if [ $? -ne 0 ]; then echo Failure: could not create initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}; exit 1; fi
echo Installing initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
cp initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX} ${EFIDIR}/
if [ $? -ne 0 ]; then echo Failure: could not install initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}; exit 1; fi
echo Instaling bzImage
cp arch/x86/boot/bzImage ${EFIDIR}/vmlinuz-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
if [ $? -ne 0 ]; then echo Failure: could not install vmlinuz-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}; exit 1; fi

# Configuring bootloader
cat <<EOF > ../loader.conf
timeout 3
default rt.conf
EOF
echo Installing loader.conf
cp ../loader.conf ${EFIDIR}/loader/loader.conf
if [ $? -ne 0 ]; then echo Failure: could not install load.conf; exit 1; fi

cat <<EOF > ../generic.conf
title Generic
linux /vmlinuz-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
initrd /initrd.img-invalid
options root=UUID=${ROOTUUID}
EOF
echo Installing generic.conf
cp ../generic.conf ${EFIDIR}/loader/entries/generic.conf
if [ $? -ne 0 ]; then echo Failure: could not install generic.conf; exit 1; fi

cat <<EOF > ../rt.conf
title Real-time
linux /vmlinuz-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
initrd /initrd.img-${KERNELMAJOR}.${KERNELMINOR}.${KERNELPATCH}-${KERNELSUFFIX}
options root=UUID=${ROOTUUID}
EOF
echo Installing rt.conf
cp ../rt.conf ${EFIDIR}/loader/entries/rt.conf
if [ $? -ne 0 ]; then echo Failure: could not install rt.conf; exit 1; fi

bootctl install
