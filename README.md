# Arduino ODBII scan tool simulator.

Transmits ODBII PIDs with vehicle sensor information upon request, using the
ELM327 protocol.

The common setup will be a set of automotive sensors connected to the Arduino
microcontroller (MCU) running as the simulator. Equipped with a Bluetooth
module, sensor information will be sent to a client when requested. The client
will generally be a smart phone running an OBDII app, such as Torque.

This program implements the aqcuisition and calculation of sensor values, and
transmission of those to the client using OBDII 
[PID structures](https://en.wikipedia.org/wiki/OBD-II_PIDs) over the ELM327
protocol.


```

        +-----+---+     +--------+        Xx     +----+
        |     |   |     |  +--+  |     Xx  XX    |----|
        |    +++  |     | -+  +- |  Xx  XX  XX   ||  ||
        | -> | |  +---> | -+  +- |   XX  X   X   ||  ||
        |    +++  |     |  +--+  |  Xx  XX  XX   ||  ||
        |     |   |     |        |     Xx  XX    |----|
        +-----+---+     +--------+        Xx     +----+

          Sensor      MCU + Bluetooth          Smartphone
                     (OBDII scan tool           or tablet
                        simulator)

```
