.. _bluetooth-mesh-onoff-sample:

Bluetooth: Mesh OnOff Model
###########################

Overview
********

This is a simple application demonstrating a Bluetooth mesh multi-element node.
Each element has a mesh onoff client and server
model which controls one of the 4 sets of buttons and LEDs .

Prior to provisioning, an unprovisioned beacon is broadcast that contains
a unique UUID. It is obtained from the device address set by Nordic in the
FICR. Each button controls the state of its
corresponding LED and does not initiate any mesh activity.

The models for button 1 and LED 1 are in the node's root element.
The 3 remaining button/LED pairs are in elements 1 through 3.
Assuming the provisioner assigns 0x100 to the root element,
the secondary elements will appear at 0x101, 0x102 and 0x103.

After provisioning, the button clients must
be configured to publish and the LED servers to subscribe.

If a LED server is provided with a publish address, it will
also publish its status on an onoff state change.


Requirements
************

This sample has been tested on the Nordic nRF52840-DK board.

Building and Running
********************

The following commands build the application.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/nrf/mesh/onoff-app
   :board: nrf52840dk_nrf52840
   :goals: build flash
   :compact:
