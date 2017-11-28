# Arduino ODBII scan tool simulator.

Transmits real-time automotive sensor information to a client smartphone
or tablet running an [ODBII](https://en.wikipedia.org/wiki/OBD-II) monitoring 
app.

The most common application is to provide real-time sensor information
for classic vehicles that either do not have an ECU or a sensor bus with
an OBDII connector. 

The common setup will be a set of automotive sensors connected to the Arduino
microcontroller (MCU) running as the simulator. Equipped with a Bluetooth
module, sensor information will be sent to a client when requested. The client
will generally be a smartphone running an OBDII app, such as 
[Torque](https://torque-bhp.com/).

This program implements the aqcuisition and calculation of sensor values, and
transmission of those to the client using OBDII 
[PID structures](https://en.wikipedia.org/wiki/OBD-II_PIDs) over the ELM327
protocol upon request.


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
