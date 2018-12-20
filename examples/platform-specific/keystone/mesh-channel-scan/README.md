## Mesh Channel Scanner

This example application demonstrates measuring RSSI from the CC1352 
on all of the IEEE.15.4g channels in the selected band.

The band is automatically selected based on the board variant used (board-conf.h)

Every 8 seconds a scan is performed. In the 915 band, the scan of 129 channels
takes about 4 seconds.

A graph of RSSI is displayed as shown in this example.  All channels that
report a measurement will show a * in the -120 to -128 bucket.


------ scan cycle duration 4 s, per channel: 31 ms ----
RSSI -----------------------------------------------------------------
-08:
-16:
-24:
-32:
-40:
-48:
-56:
-64:
-72:
-80:
-88:
-96:
-104:
-112:
-120:  * *      **  *  *** ** ** **       **   *  ***  ** *      *  **
-128: *****************************************************************
     -----------------------------------------------------------------
     0                                                               64
RSSI ----------------------------------------------------------------
-08:
-16:
-24:
-32:
-40:
-48:
-56:
-64:
-72:
-80:
-88:
-96:
-104:
-112:
-120:      *  *  ** *      * *     * ***   *   ** ****      * **    **
-128: ****************************************************************
     ----------------------------------------------------------------
     65                                                              128


