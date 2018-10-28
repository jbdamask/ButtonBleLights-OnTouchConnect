# ButtonBleLights-OnTouchConnect
Code for Adafruit Feather Bluefruit 32u4 with button. This code allows for the control of attached NeoPixels by either button push or Bluetooth Low Energy. Button push events are packaged into a payload and written to UART TX for use by listeners.

This is designed to work with my [PiHubBleAwsIoT|https://github.com/jbdamask/PiHubBleAwsIoT] project. The point is that when a BLE device attached to one thing (Thing 1) is touched, it published the envent to a queue monitored by another thing (Thing 2). Thing 2 gets the state and transmits to it's connected BLE devices. This will let me simulate phone call-like operations of RINGING, CONNECTED and DISCONNECTED

Also uses NeoPixel RBGW

## Demo
https://www.youtube.com/watch?v=Ffhj39ibOfI

## ToDo
* Write at speed of thought...care only about functionality. Once it all works, clean up the code
