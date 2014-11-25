^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_span_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2014-11-25)
------------------
* Add capture from INS PPP, default value for IMU rate, fix test, add docs to example launcher.
* Throttle wheelvelocity commands to 1Hz, as recommended by Novatel.
* Fix velx/y mixup, add diagnostic publisher.
* BESTPOSB -> BESTPOS, add diagnostic constants and shim class.
* Treat altitude as positive-up.
* Update messages to include header, fix header to match documentation.
* Add wheel velocity handler.
* Contributors: Mike Purvis

0.2.0 (2014-10-29)
------------------
* Re-work the publisher class.
* Add a pcap for a SPAN fix.
* Tweaks to topic names.
* Add working rostest, fixes from pepify.
* Add roslaunch file check for example launcher.
* Add roslint test.
* Contributors: Mike Purvis

0.1.0 (2014-10-24)
------------------
* Consolidation and reorganization of novatelcentral driver prepared by NovAtel
  and published here: https://github.com/NovAtel-inc/rosnovatel
* Contributors: Mike Purvis
