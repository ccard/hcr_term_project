^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concert_service_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-08-26)
------------------
* teleop service uses video_teleop
* update publisher queue_size to avoid warning in indigo.
* specify capture timeout as a parameter for concert teleop.
* parameterised velocities for concert teleop.
* upgrades for the rapp manager launch overhaul.
* concert_teleop_app -> concert_teleop
* remove rocon_scheduler_requests from run_depend
* expose available and preemptible teleop robots
* usable configuration for service priorities.
* rocon_scheduler_requests -> concert_scheduling/concert_scheduler_requests
* make sure lock is available before functions use it, fixes `#6 <https://github.com/robotics-in-concert/concert_services/issues/6>`_.
* rocon_service -> concert_service
* Contributors: Daniel Stonier, Jihoon Lee
