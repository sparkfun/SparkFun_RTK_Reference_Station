SparkFun RTK Reference Station
========================================

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/22429"><img src="https://cdn.sparkfun.com/assets/parts/2/2/5/2/3/SparkFun_GNSS_RTK_Reference_Station_-_05.jpg"></a></td>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/22429">SparkFun RTK Reference Station (GPS-22429)</a></td>
  </tr>
</table>

The RTK Reference Station from SparkFun is our most capable GNSS receiver and logger to date. It's your one stop shop for high precision geolocation, surveying and time reference needs. For basic users, it is incredibly easy to get up and running; for advanced users, the Reference Station is a flexible and powerful tool. We took everything we learned while developing our other RTK products, refined it and the Reference Station is the end result.

Is this just a [RTK Facet](https://www.sparkfun.com/products/19984) in a metal box? Oh no! It is _so_ much more... Sure, it is based on the same ESP32-WROOM processor and u-blox ZED-F9P multi-band GNSS module as the Facet. And it runs the same core firmware as the rest of the RTK product family. But there the similarities end. The Reference Station has 10/100 Mbps Ethernet connectivity provided by a WIZnet W5500 and it can be powered by Power-over-Ethernet (PoE) too. With just a few minutes of setup and survey-in, your Reference Station can be serving RTCM correction data to an NTRIP caster of your choice, all via Ethernet!

Need an affordable Network Time Protocol time server for your Ethernet network? We've got you covered. The Reference Station can act as a NTP server. It supports DHCP by default, but you can give it a fixed IP address if you want to. DNS, gateway and subnet mask are all configurable too.

The Reference Station gets a big speed boost too. The microSD card socket is connected via full 4-bit SDIO instead of the usual SPI, providing an order or magnitude improvement in read and write speeds. Similarly, the u-blox ZED-F9P GNSS module is connected via SPI instead of the usual I2C, also providing an order of magnitude improvement in data transfer speeds. Need to log RAWX and SFRBX at 20Hz? You can with the Reference Station!

Repository Contents
-------------------

* [**/Reference_Station_Test_Sketches**](/Reference_Station_Test_Sketches/) - Stand-alone Arduino test sketches for the Reference Station
* [**/Hardware**](/Hardware/) - Eagle PCB files, schematic and dimensions
* [**/Enclosure**](/Enclosure/) - Mechanical drawings for the custom extruded aluminium case and machined panels
* [**/Front_Panel**](/Front_Panel/) - Eagle PCB files for the prototype (fit-check) front panel
* [**/Rear_Panel**](/Rear_Panel/) - Eagle PCB files for the prototype (fit-check) rear panel
* [**/Front_Panel_Sticker**](/Front_Panel_Sticker/) - Drawings and prototype (fit-check) files for the front sticker
* [**/Rear_Panel_Sticker**](/Rear_Panel_Sticker/) - Drawings and prototype (fit-check) files for the rear sticker
* [**/Sticker_Alignment_Jig__Panel**](/Sticker_Alignment_Jig__Panel/) - Eagle PCB files for the two-piece sticker alignment jig
* [**/Sticker_Alignment_Jig__Sticker**](/Sticker_Alignment_Jig__Sticker/) - Eagle PCB files for the two-piece sticker alignment jig

Documentation
--------------

* **[RTK Product Manual](https://docs.sparkfun.com/SparkFun_RTK_Firmware/)** - A detailed guide describing all the various software features of the RTK product line. Essentially it is a manual for the firmware used by the RTK Reference Station.
* **[RTK Reference Station Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-rtk-reference-station-hookup-guide)** - Hookup guide for the SparkFun RTK Reference Station.

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
