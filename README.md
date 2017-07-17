# Swarmlist List-Based

Compiling
---------

    $ mkdir build
    $ cd build
    $ cmake ../src
    $ make
    $ sudo make install

Running the experiments
-----------------------

You must have [argos3](http://www.argos-sim.info) installed, as well as its
[kilobot extension](http://www.argos-sim.info/extensions.php). Inside the directory you created in the last step, execute either:

    $ make run # To run jobs locally

Or:

    $ make submit # To run jobs on a server that uses qsub, such as the Mammoth supercomputer.