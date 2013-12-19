exploration-hh
==============

Our exploration-node

This node is called, if the Smart Environment wants the robot to search for a person.
At the moment, this node retreives a list of possible places from the database - orders them in the best order and calls move_base to move the robot there.

This node will control the whole exploration process.
Long term goal is to be able to operate without given places and/or operate on a probability-map.

The code is licenced unter GPL v2.
If you have any questions, don't hestitate to ask.
