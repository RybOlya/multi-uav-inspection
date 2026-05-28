"""Swarm IoT — operator-driven UAV swarm in NVIDIA Isaac Sim + Pegasus.

The runtime composes only literature-grounded components:

* Pegasus Simulator [22] for the Isaac Sim multi-vehicle infrastructure.
* Pegasus' nonlinear trajectory-tracking controller, implementing the
  control law of Mellinger & Kumar (2011) [25] / Pinto, Guerreiro,
  Cunha (2021) [24], with the canonical SE(3) tracker of Lee, Leok,
  McClamroch (2010) [9] as theoretical anchor.
* Quintic point-to-point time scaling per Lynch & Park, *Modern
  Robotics* (2017) [26].
* Boustrophedon Cellular Decomposition (single-cell sweep) per
  Choset (2000) [4] for ``scan`` commands.
* MQTT v5 (OASIS [17]) for the operator command / telemetry channel.

We do *not* ship a SLAM stack of our own; the controller consumes Pegasus'
ground-truth ``State``. For real GPS-denied deployment, the natural
substitution is a visual-inertial stack such as ORB-SLAM3 [7] or
VINS-Fusion [8].
"""

__version__ = "0.4.0"
