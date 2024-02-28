# SparkFun u-blox PointPerfect Library

The u-blox PointPerfect Library (PPL) is a truly wonderful piece of code.

If you feed the PPL with PointPerfect SPARTN / PMP atmospheric correction data, from a L-Band or an IP source,
and also feed it with your approximate position (GPGGA) and GNSS ephemeris data (UBX-SFRBX or RTCM),
it will generate RTCM correction data for your location, which your GNSS can use to achieve RTK FIXED solutions.

This is a very big deal.

Because the PPL outputs standard-format RTCM correction data, it allows non-u-blox (non-SPARTN) GNSS receivers
to achieve RTK FIXED solutions too. So long as you have a valid u-blox Thingstream PointPerfect L-Band or L-Band + IP
subscription, and have access to the current SPARTN encryption key, you can use your PointPerfect corrections on any
RTK-capable receiver.

If you think about what the PPL is doing, you will appreciate just how sophisticated it is. Based on the GNSS ephemeris
data and the SPARTN / PMP atmospheric correction data, it generates RTCM corrections as if they were coming from a
_virual_ RTK Base close to your actual location. Essentially, it is doing the exact same thing internally as your u-blox
SPARTN-capable RTK GNSS receiver. But by outputting the corrections as RTCM, you can use them on non-SPARTN receivers.

The u-blox PPL is normally distributed under a Non-Disclosure Agreement - see [LICENSE.md](./LICENSE.md) for more details.
However, we requested and were very kindly given permission to publish the ESP32 version of the PPL compiled binary
(Support Request #6030).

We are using the PPL in the new SparkFun RTK Facet mosaic (Tri-Band, L1/L2/L5/L-Band) based on the Septentrio mosaic-X5
and using its built-in L-Band receiver. It will come with a 12 month subscription to PointPerfect, like our existing Facet
L-Band. The decryption key will be delivered seamlessly via One Touch Provisioning.

We are also using the PPL in the new SparkFun RTK Torch (Tri-Band) based around the Unicore UM980. Here SPARTN data could
be delivered by MQTT over WiFi, or via Bluetooth with your phone providing the data connection to Thingstream.

## Repository Contents

* [/examples](./examples/) - Example sketches which demonstrate how to use the PPL on different hardware. The hardware and connections are described in each example
* [/src](./src/) - PPL header files and pre-compiled ESP32 binary - by kind permission of u-blox
* [keywords.txt](./keywords.txt) - Keywords from the binary that will be highlighted in the Arduino IDE
* [library.properties](./library.properties) - Library properties for the Arduino package manager

## License Information

This library is _not_ open source. The PPL header files and pre-compiled ESP32 binary are reproduced by kind permission of u-blox.

Please see [LICENSE.md](./LICENSE.md) for details.

* Your friends at SparkFun

