ConsoleCast front-end example
=======================

This example will show the main display reading of the Owon XDM2041 in your browser.

![XDM2041](https://consolecast.nickstick.nl/assets/img/owon.png)

This is what you need to do:

- Attach ConsoleCast to the XDM2041 using a serial connection

You can use a normal (short) RJ45 patch cable and an adapter for the Owon DB9 port. The DB9 on the Owon is a male connector so you will need a female plug (or gender changer). Having the jumpers on ConsoleCast set in "Patch" config (the default) you need the following line connections:

| DB9 | RJ45 | Remark |
| --- | --- | --- |
| 2 | 3 |
| 3 | 6 |
| 5 | 4 and/or 5 |
| (7) | (3) | optional
| (8) | (3) | optional

For more info see [consolecast.nickstick.nl](https://consolecast.nickstick.nl)

- Switch on the Owon and ConsoleCast
- Connect your computer to the ConsoleCast Access Point "ConsoleCast_AP"
- In your browser open this example html file as a local file like this: "file:///_your_local_directory_/xdm2041.html"
- The JS code will now connect to and communicate with ConsoleCast and show the main display reading of the Owon

For more SCPI commands for the XDM2041, you can find the programming manual for the complete command-set [here](https://www.technica-m.ru/upload/support_ext/owon_xdm2041_digital_multimeter_programming_manual.pdf). You can use the on-board ConsoleCast terminal for testing out these commands.
