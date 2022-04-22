^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ntrip_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-04-22)
------------------
* ROS Ntrip Version Configuration (`#8 <https://github.com/LORD-MicroStrain/ntrip_client/issues/8>`_)
  * No longer sends Ntrip-Version header if not specified
  * Adds some hopefully helpful debug logging
  * Properly handles responses that have both a failure response and success response
  * Adds ability to print debug logs via launch parameter
* Contributors: robbiefish

1.0.1 (2022-02-10)
------------------
* Checks if there is a * character in the string before parsing fully
* Contributors: robbiefish

1.0.0 (2021-12-08)
------------------
* Initial implementation of ROS NTRIP client
* Adds ability to cache packets if they do contain some of a mesage but not the whole thing
* Contributors: drobb257, nathanmillerparker, robbiefish
