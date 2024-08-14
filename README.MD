# Huawei Matebook E Go (8cx Gen 3)

This repository contains workarounds for the Huawei Matebook E Go (8cx Gen 3) to support Linux.

This project mainly refers to the following projects:
* [jhovold](https://github.com/jhovold/linux): who adds patches to the mainline to support X13s and SC8280XP (8cx Gen 3).
* [steev](https://github.com/steev/linux/tree/lenovo-x13s-v6.10-rc3): who maintains the kernel for X13s.
* [matalama80td3l](https://github.com/matalama80td3l/matebook-e-go-boot-works): who migrates the device tree from X13s' for the Huawei Matebook E Go.
* [chenxuecong2](https://github.com/chenxuecong2/linux-huawei-matebook-e-go): who migrates the device tree from X13s' for the Huawei Matebook E Go.

## Disclaimer
I am just an amateur, a Linux noob, not educated with formal CS courses. You should never believe what I say below, except this sentence. I am not responsible for your actions; you do this all at your own risk!

## Kernel
Use steev or jhovold one with default config and `simplefb` enabled. BTW, I have a simple workflow for it.

Kernel parameter: `clk_ignore_unused pd_ignore_unused arm64.nopauth efi=noruntime`

## Device Tree
The DT works with steev's latest branch. You should extract the firmwares required by DT, and get them well placed, you can find them in the Windows OS `DriverStore` directory.

## Feature Support

Refer to [this](https://github.com/jhovold/linux/wiki/X13s) as well.

With kernel version `6.10-rc3`.

| Feature | Status | Note |
| ------- | ------ | ---- |
| Audio  | works | see [below](#audio) |
| Backlight | x |  |
| Battery | x | Huawei uses a different method from [this](https://github.com/jhovold/linux/commit/5999451b4320ec171cf2d29dc5c479db0901a099). |
| Bluetooth | untested |  |
| Camera | x | |
| CPUfreq | works | |
| Display | x | [There](https://linaro.github.io/msm/soc/sc8280xp) is no DSI driver for SC8280XP now. |
| EFI variables | works | with the [patch](./0001_enable_qseecom.patch). |
| Fingerprint reader | x | introduce from [commit](https://github.com/torvalds/linux/commit/d280fe309b88b337a9b26f96b7a9d9aa6d6c3a0f). |
| GPU | unsure | won't know until we get the display driver working. |
| Hibernation | x | |
| Keyboard | works | |
| Keyboard fn keys | partial | |
| NVMe SSD | works | |
| PCIe (x4) | works | |
| Remoteproc | works | |
| RTC | works | |
| Suspend | untested |  |
| Thermal sensors | works | |
| Touchpad | works | |
| Touchscreen | partial | weird behavior |
| TPM | x | |
| USB | works | |
| USB-PD | unsure | not sure if it triggers the PD protocol |
| USB-C DisplayPort Alt Mode | unsure | unknown for now. Only the main display is available at the initial system bootup. |
| Video acceleration | unsure | depends on the main display driver status |
| Virtualisation | x | |
| Volumn keys | works | |
| Watchdog | works | |
| Wi-Fi | works | use the 2.4 GHz band for stability. |

## Audio
Recently, use the X13s' profile. \
`sed -i 's/LENOVO.*ThinkPad X13s.*/LENOVO.*ThinkPad X13s.*|HUAWEI.*MateBook E.*\"/' /usr/share/alsa/ucm2/Qualcomm/sc8280xp/sc8280xp.conf`

