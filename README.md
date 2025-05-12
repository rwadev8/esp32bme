# esp32bme
bme688 sensor tests on an esp32s3 dev board

- based on esp32temp
- first tests with the adafruit bme680 library

readout interval

it seems the read interval greatly influences the resistance value.
with a target temp of 300 C and a an interval of 100 ms, when reading every 2 sec the sensor returns around 80 k Ohm, at 5 sec we are at 55 k Ohm and at 10 sec around 38 k Ohm, in the same environment
