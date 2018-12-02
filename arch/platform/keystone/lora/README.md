## LoRaMac-Node adapted for Contiki-NG and the Keystone platform.

Source: `https://github.com/ThisIsIoT/Contiki-LoRaMac/tree/ee2f065c596d629973fa45a9f345042d7106d19d`
Branch: develop.

**Summary of changes**

* Flattened file heirarchy to work better with Contiki build system. 
* Prepend `lora-` to files that might conflict with Contiki build system namespace.
* `Timer` adapted to use Contiki `ctimer`.
* All board-specific adaptations located in `sx1262-board.c`.
* Only files relevant to LoRaWAN stack operation included.
