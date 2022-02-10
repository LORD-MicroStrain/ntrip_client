^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ntrip_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2022-02-10)
------------------
* Checks if there is a * character in the string before parsing fully
* Contributors: robbiefish

1.0.1 (2022-01-20)
------------------
* Replaces ROS timer destroy methods with ROS2 methods and removes extra destroy
* Contributors: robbiefish

1.0.0 (2021-12-09)
------------------
* Initial implementation of ROS2 NTRIP client
* Adds ability to cache packets if they do contain some of a mesage but not the whole thing
* Contributors: drobb257, nathanmillerparker, robbiefish
