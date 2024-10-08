# Huawei Matebook E Go (8cx Gen 3)

This repository contains workarounds for the Huawei Matebook E Go (8cx Gen 3) to support Linux.

This project mainly refers to the following projects:
* [jhovold](https://github.com/jhovold/linux): who adds patches to the mainline to support X13s and SC8280XP (8cx Gen 3).
* [steev](https://github.com/steev/linux): who maintains the kernel for X13s.
* [linaro team](https://patchwork.kernel.org/project/linux-pm/cover/20240614-yoga-ec-driver-v7-0-9f0b9b40ae76@linaro.org/): who authored the Lenovo c630 EC driver, and showed an example how to write auxiliary device modules.
* [Nikita Travkin](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/platform/arm64/acer-aspire1-ec.c): who authored the very first Acer aspire1(sc7180) i2c EC driver.
* [aymanbagabas](https://github.com/aymanbagabas/Huawei-WMI): who authored Huawei laptop wmi driver and showed the meaning of ACPI identifiers like GBTT, SBTT, etc
* [NewWheat](https://gitlab.com/New-Wheat/linux-for-huawei-matebook-e-2019): who helped me find EC interrupt, and offered one copy of i2c EC driver for reference.
* [matalama80td3l](https://github.com/matalama80td3l/matebook-e-go-boot-works): who migrated the initial device tree from X13s' for the Huawei Matebook E Go.
* [chenxuecong2](https://github.com/chenxuecong2/linux-huawei-matebook-e-go): who migrated the initial device tree from X13s' for the Huawei Matebook E Go.

## Disclaimer
I am just an amateur, a Linux noob, not educated with formal CS courses. You should never believe what I say below, except this sentence. I am not responsible for your actions; you do this all at your own risk!

## Kernel
Use steev or jhovold one with default config and `simplefb` enabled. BTW, I have a simple workflow for it.

Kernel parameter: `clk_ignore_unused pd_ignore_unused arm64.nopauth efi=noruntime`

## Device Tree
The DT works with steev's latest branch. You should extract the firmwares required by DT, and get them well placed, you can find them in the Windows OS `DriverStore` directory.

## Feature Support

Refer to [this](https://github.com/jhovold/linux/wiki/X13s) as well.

With kernel version `6.11-rc2` by [steev](https://github.com/steev/linux/).

| Feature | Status | Note |
| ------- | ------ | ---- |
| Audio  | works | see [below](#audio) |
| Backlight | x |  |
| Battery | works | with my ec driver, cycle count is also supported |
| Bluetooth | partial | paired and connected with my android phone, but keep disconnected with my LinkBuds S |
| Camera | x | |
| CPUfreq | works | |
| Display | x | [There](https://linaro.github.io/msm/soc/sc8280xp) is no DSI driver for SC8280XP now. I added some patches to *patch sets* folder, but still not work |
| Display sensors | x | ACPI call it so, for rotation detect, light detect, etc. Added pinctrl nodes, slpi nodes, but how can we access it? |
| EFI variables | works | with the [patch](./patch\ sets/fix/0001-firmware-qcom-uefisecapp-add-Huawei-Matebook-E-GO.patch). |
| EC(WIP) | partial | Battery and adapter status, maybe modern standby |
| Fingerprint reader | x | mbn firmwares related.(check your Windows driver repo) x13s one was introduce from [commit](https://github.com/torvalds/linux/commit/d280fe309b88b337a9b26f96b7a9d9aa6d6c3a0f). |
| GPU | works | It works, but the dsi output not, the image is actually here, check [this](./feature_spoliers/gpu.png) |
| Hibernation | x | |
| Keyboard | works | |
| Keyboard fn keys | partial | |
| NVMe SSD | works | |
| PCIe (x4) | works | |
| Power key | works | |
| Remoteproc | works | adsp, cdsp, sdsp(sensors related, see [below](./#SLPI)) |
| RTC | works | |
| Suspend | works | s2idle |
| Thermal sensors | works | partial I think |
| Touchpad | works | |
| Touchscreen | partial | weird behavior |
| TPM | x | |
| USB | works | |
| USB-PD | unsure | not sure if it triggers the PD protocol |
| USB-C DisplayPort Alt Mode | partial | edid and modes can be read, but there is no video signal. maybe one day EC done, huawei does not use the pmic glink framework, it uses interface from EC |
| Video acceleration | unsure | depends on the main display driver status |
| Virtualisation | x | |
| Volumn keys | works | |
| Watchdog | works | |
| Wi-Fi | works | use the 2.4 GHz band for stability. |
| WMI related features| partial | charging control, FN lock, smart charge, thermal sensors and so on(undiscoverd things due to its meaning is unknown) |

## Audio
Recently, use the X13s' profile. \
`sed -i 's/LENOVO.*ThinkPad X13s.*/LENOVO.*ThinkPad X13s.*|HUAWEI.*MateBook E.*\"/' /usr/share/alsa/ucm2/Qualcomm/sc8280xp/sc8280xp.conf`

## Charging control
`echo 'start stop' /sys/devices/platform/gaokun-wmi/charge_control_thresholds`

Clarification from [TLP](https://linrunner.de/tlp/faq/battery.html).
> Start charge threshold (START_CHARGE_THRESH_BATx): battery charge level below which charging will begin when connecting the charger.

> Stop charge threshold (STOP_CHARGE_TRESH_BATx): battery charge level above which charging will stop while the charger is connected.

The values would not be saved after you shut down and plugged out, the default values are set to `0 0` at next boot, you can use `udev` to set the default values like `55 60`,
```
ACTION=="add", SUBSYSTEM=="platform", KERNEL=="gaokun-wmi", ATTR{charge_control_thresholds}="55 60"
```

## Camera
* front: hi846 or s5k4h7
* rear: ov13b10 or s5k3l6

## Display
There are too many unknowns(dsi support, dual dsi with video mode dsc, gpio, power supply, backlight, etc), no actual progress can be confirmed, I have uploaded all my progress, if you have an idea, then go ahead.

## FN Lock
`echo 0/1 > /sys/devices/platform/gaokun-wmi/fn_lock_state` \
`0`: disable the fn key(hotkey first, such as press `F1` to decrease brightness) \
`1`: enable the fn key(function key first), you can only trigger hotkey with `Fn + F1-12` \
BTW, you can press `Fn` key(light on) to reverse it temporarily.

## Smart charging
I know less about the meaning of arguments, but it is implemented, check and do your own investigation, feel free to give me feedback \
`cat /sys/devices/platform/gaokun-wmi/smart_charge` \
`cat /sys/devices/platform/gaokun-wmi/smart_charge_param`

`smart_charge`: default are 4 72 40 70, correspond to charging mode, delay, start, stop, after 72 hours, charging level is limited to 70%.

`smart_charge_param`: enable or not? I am not sure.

## Thermal Sensors from EC
I don't know these sensors for which devices, but you can read them \
`cat /sys/devices/platform/gaokun-wmi/temperature`

## Features spoilers
[battery](feature_spoliers/battery_features.png)

## SLPI
SLPI(Sensor Low Power Island), [more details](https://lore.kernel.org/linux-devicetree/20230406173148.28309-1-me@dylanvanassche.be/).
```
[    2.606897] remoteproc remoteproc2: remote processor 2400000.remoteproc is now up
[    2.613505] qcom,fastrpc 2400000.remoteproc:glink-edge.fastrpcglink-apps-dsp.-1.-1: no reserved DMA memory for FASTRPC
```
The firmware of slpi can be loaded, you can also find `/dev/fastrpc-sdsp`, not support `IOMMU` for now. Even though, how can I access sensors?

## Progress
Suspended(Since 08/25/2024), due to real life things.

## Help wanted
I am occupied with real life things, I spend much free time on it, if someone can help me test features, that will be awesome. Also, as I am an amateur, I am happy to receive suggestions from coding, ACPI/DSDT/SSDT reversing, working mechanism of UCSI, DSI panel, etc.
