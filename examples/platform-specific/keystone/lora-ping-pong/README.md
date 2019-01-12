## SX1262 radio testing app.

This application uses the LoRa transceiver (SX1262) on the
IoT.Keystone Innovator Board to send and receive PING and PONG messages
with another IoT.Keystone board running the same application.

The application is auto-configuring, so all that needs to be done is
to load the application on both boards and turn them loose.  You should see the
green and red LEDs alternately toggle at approximately a 1 Hz rate if
nodes are able to communicate with each other.
 
How to use:
Both nodes start up as a master and will alternate between 
sending a PING message and listening for responses.
When a node receives a ping message, it turns into a slave node
and will respond with a PONG for each PING it receives.
   
The RED LED is toggled for each message sent (be it a PING or PONG)
The GREEN LED is toggled for each message received.

This application is based on the apps/ping-pong example in the
Lora-net/LoRaMac-node GitHub repo.

The channel, spreading factor, bandwidth and so on can be configured
in the application.





