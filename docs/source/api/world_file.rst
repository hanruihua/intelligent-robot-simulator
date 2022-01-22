======================
API for the world file
======================

Introduction
~~~~~~~~~~~~

The world file is used to formulate the environment with yaml format. A comment world file: **empty.yaml** is shown as follows:

.. code-block:: yaml
    :linenos:

    world:
        world_height: 10
        world_width: 10
        offset_x: 0
        offset_y: 0
        step_time: 0.1
        xy_resolution: 0.01 # meter
        yaw_resolution: 5 # degree
        world_map: 'map_100_100.png'

Most of the parameters can be also set by the env function directly, where will cover the set in this file. 


API
~~~~~












