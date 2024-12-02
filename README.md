# skeeter
- Plastic welding spoolgun
- Over Engineered hot glue gun
- Under Engineered 3D printer (handheld)
- 3D pen on steroids

This tool is designed to make the welding together of 3D printed parts go quickly.
It operates from DeWalt 20V battery power or a 24V power supply; there are two handle variants you can print depending on how you want to power it.

![image](https://github.com/user-attachments/assets/5bf8b520-0633-436e-b935-0309651d7ec7)


BOM:
1. 1X pair of printed parts
2. 1x [DeWalt Battery holder](https://www.amazon.com/gp/product/B0C282C27R/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) (if you want battery powered)
3. 1x Orbiter 2 extruder
4. 1x 2510 fan 24V
5. 1x [Pololu DC motor](https://www.pololu.com/product/3491)
6. 1x [Peopoly Lancer Long hotend](https://peopoly.net/products/magneto-x-lancer-melt-zone?variant=49972216135962)
7. 1x [FA2-16/WEK drill trigger motor controller](https://www.amazon.com/gp/product/B08PKZ464X/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
8. 1x [MKS THR42 3d printer toolhead board](https://makerbase3d.com/product/mks-thr36-thr42-board/?srsltid=AfmBOoqB9_kss1FwxlIhkWJhzdaipdb-LNaB0u1sYBQivyn0e_-DFAhm)
9. 4x 8-32 x0.5" countersunk screws
10. 4X M3x5mm standoffs
11. assortment of misc M3 screws
12. 18 awg wire 

Instructions:
1. print the parts. handle1.stl is for non-battery power and handle2 is for DeWalt battery power; you don't need both, just one or the other.
2. remove the stepper motor from the orbiter extruder
3. fabricate/machine a motor adapter plate and shaft adapter to mount the Pololu motor in place of the orbiter stepper
4. tap holes in the 3D printed parts. The 4 holes that hold the top part to the bottom are 8-32 and everything else is M3
5. assemble everything. T
6. Connect to the gun to PC via USB and flash the skeeter.uf2 file to the THR42 board via [MKS's firmware update instructions](https://github.com/makerbase-mks/MKS-THR36-THR42-UTC?tab=readme-ov-file#thr3642-firmware-update) . Ignore all the stuff about klipper config; this is not klipper firmware, it is a totally separate firmware that I wrote, to control the THR42 board as it's own standalone thing.
7. With the gun connected, to PC, connect to it using a terminal app like Putty or Arduino serial monitor. Send it a temperature (C) setpoint command like "S220" (without quotes) to set 220C.
8. It shouldn't need any PID tune adjustments if using Peopoly Lancer Long, but if using something else you can change the PID parameters by typing (for example): [Pxxx.xxx - Set PID Proportional constant. Ex: P2.25], [Ixxx.xxx - Set PID Integral constant. Ex: I0.25], [Dxxx.xxx - Set PID Derivative constant. Ex: D1.525]
9. Go shoot plastic!

The skeeter.uf2 firmware file is generated by Arduino IDE using the skeeter.ino file which you are welcome to modify however you want.

**NOTE: there is no low voltage cutout circuit on this tool so if you use the battery powered version, it is 100% capable of discharging your battery down to damaging levels!**

