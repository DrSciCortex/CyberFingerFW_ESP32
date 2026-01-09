# CyberFingerFW_ESP32
ESP32 Firmware for the SciCortex CyberFinger - a device that combines the versitility of a VR controller with the freedom and social expressiveness of optical hand tracking.
CyberFinger was designed for creative, research, enterprise and education use cases in Resonite.

<img src="https://github.com/user-attachments/assets/4be3dc2d-3672-4cfb-9949-469d33995f76" width="300">


To use this firmware, you need a pair of CyberFingers;
see https://scicortex.com/products/cyberfinger-v1-0-beta

or you could build your own:
https://scicortex.com/products/diy-cyberfinger-kit

and you need the CyberFingerMod for Resonite https://github.com/DrSciCortex/CyberFingerMod.
Further install and setup steps are described there. 

👉 **[Watch the CyberFinger demo on YouTube](https://www.youtube.com/watch?v=8n6NSjitQ9c)

[![Video Title](https://img.youtube.com/vi/8n6NSjitQ9c/0.jpg)](https://www.youtube.com/watch?v=8n6NSjitQ9c)


# Configuring the firmware over USB

The CyberFinger firmware supports configuring several aspects, including button GPIO pin assignments, via a JSON payload provided over USB at boot time.  These parameters are then stored in NVS on the ESP32.
A python script to read an write the JSON configs is provided under : config/quick_cfg.py

How to use it?
Firt you'll need a working python environment. Install the dependencies:
```
pip install pyserial
```

Power off the device while USB-C connected.
To write a json config (here the default right hand controller config), run this command on windows:
```
python.exe .\quick_cfg.py <serial-port> --mode write --json right_model1000r1.json
```
or this command on linux:
```
python quick_cfg.py <serial-port> --mode write --json right_model1000r1.json
```
where serial-port is something like COM9 on Windows, or /dev/ttyACM0 on Linux 

If successful, the output should look something like this:
```
> python.exe .\quick_cfg.py COM9 --mode write --json right_model1000r1.json
Waiting for COM9 ... (Ctrl+C to quit)
Writing...
JSON: {"pins":{"buttonAX":43,"buttonBY":44,"buttonBP":39,"buttonST":38,"buttonStartSelect":0,"joyX":17,"joyY":18},"axes":{"invertX":true,"invertY":true},"config":{"play_sound":true,"right_not_left":true,"boot_debug":false},"wifi_channel":11,"esp_interval":11363}

Sent 257/257 bytes
provisionOrLoad - got cmd: WRITE
Payload: {"pins":{"buttonAX":43,"buttonBY":44,"buttonBP":39,"buttonST":38,"buttonStartSelect":0,"joyX":17,"joyY":18},"axes":{"invertX":true,"invertY":true},"config":{"play_sound":true,"right_not_left":true,"boot_debug":false},"wifi_channel":11,"esp_interval":11363}
OK
Successfully wrote config.
```

You can also read the config as follows:
```
python quick_cfg.py /dev/ttyACM0 --mode read --out current.json
```

# License

This CyberFinger firmware is licensed under the GNU General Public License v3.0 only (GPL-3.0-only).

Unless otherwise noted in individual source file headers, all source code in this
repository is licensed under GPL-3.0-only. Some files are included under compatible
permissive licenses (e.g. Apache-2.0 and CC0-1.0); these exceptions are clearly
identified in the relevant file headers and summarized in the NOTICE file.

The GPL-3.0 license applies to the firmware as a whole. Any redistribution of this
software—whether in source or binary form, including distribution in physical
devices—must comply with the terms of GPL-3.0, including the obligation to provide
corresponding source code and installation information for modified versions.

The full license text is provided in the LICENSE file. Third-party license texts are
provided in the LICENSES/ directory or in the respective library subdirectories.

# Contributing
By contributing to this project, you agree to the Contributor License
Agreement in CLA.md.
