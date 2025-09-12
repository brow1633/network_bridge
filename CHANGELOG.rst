^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package network_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2025-09-11)
------------------
* Merge pull request `#12 <https://github.com/brow1633/network_bridge/issues/12>`_ from brow1633/robustify
* Guard against querying graph after context goes invalid
* make some tests/shutdown more robust
* force client to start after server in TCP test (`#11 <https://github.com/brow1633/network_bridge/issues/11>`_)
  * force client to start after server
  * linting
  * no more python linting.
* Handling of large objects and clean management of reconnections (`#7 <https://github.com/brow1633/network_bridge/issues/7>`_)
  * Added queuing mechanism for reception and garanteed transmission
  * Added receive queue
  * Added proper server reconnect
  * Added logger for topic creation
  * Fixed client reconnect
  * Implemented PR request
  ---------
  Co-authored-by: Cedric Pradalier <cedric.pradalier@georgiatech-metz.fr>
  Co-authored-by: Ethan Brown <97919387+brow1633@users.noreply.github.com>
* Merge pull request `#9 <https://github.com/brow1633/network_bridge/issues/9>`_ from brow1633/8-fix-ci
  update cache version, include kilted, linting
* remove whitepsace
* Merge branch 'main' into 8-fix-ci
* Fix linting error
* update cache version, include kilted
* Contributors: Cedric Pradalier, Ethan Brown

1.0.2 (2024-07-10)
------------------
* Move launch_testing_ament_cmake requirement to if testing block
* Contributors: Ethan Brown

1.0.1 (2024-07-10)
------------------
* Merge branch 'main' of github.com:brow1633/tether
* Updated package.xml deps
* Update README.md
* Add release to CI, turn off fail fast
* Update README.md
* add pkg-config
* CI
* Python linting
* Create CI.yml
* Add boost system for asio
* Remove un-needed from package.xml
* Dependency and CMake Changes
* Remove Boost as dependency
* Added const for jazzy
* Fixed package.xml deps
* Change package name
* Use bit_cast instead of reinterpret_cast
* Fix TCP Packet Framing
* linting
* Merge branch 'main' of github.com:brow1633/tether
* Updated license
* Update README.md
* Contributors: Ethan Brown, brow1633

1.0.0 (2024-05-31)
------------------
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Added subscription namespaces, docs
* Added license to one file
* Updated readme
* init commit
* Initial commit
* Contributors: Ethan Brown, brow1633
