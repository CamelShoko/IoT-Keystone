## LoRaWAN stack for Contiki-NG

Binds the Semtech LoRaWAN stack as a MAC layer in Contiki-NG's 6LoWPAN IPv6 network stack. 

That's right: transport IPv6 UDP packets over LoRaWAN!  Or, use the LoRaWAN MAC directly
in an application via a simple packet send/receive interface.

This stack requires a platform that implements the radio driver and
system support services like timers.

We have modified this stack from what is on GitHub.
We have changed the header names so that they are unique within the
Contik-NG build system.

The stack is managed and accessed through a Contiki process implemented
in LoRaMacContiki.




