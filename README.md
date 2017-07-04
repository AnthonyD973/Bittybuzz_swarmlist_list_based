# Swarmlist List-Based

Compiling
-------------

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Running an experiment
-----------------------

You must have [argos3](http://www.argos-sim.info) installed. Go to the git repository's root, and execute:

    $ argos3 -c src/experiments/<some_file.argos>

For example:

    $ argos3 -c src/experiments/swarmlist_sim.argos
