ELEC424-Lab03_IB_scheduling
====================

A project for the STM chip to execute a set of tasks defined by the provided static library lab04_tasks.a.

The tasks of lab04_tasks.a are time sensitive, and must be executed within a given interval. Each function has a given priority that corresponds to how critical it is; for example, checkEmergency() is more important that logDebugInfo(). The goal is to execute each function at least once during within its designated interval. We assume that all tasks, except for calculateOrientation(), execute without significant delay. calculateOrientation() simulates heavy floating point math and will take some time to complete.

The lab04_tasks.h shows details about the tasks, their priorities (the smaller the higher), and execution frequency requirements. 

To test your scheduling algorithm, we can use the function updatePid(), which will check if the time constraints are met. If the scheduler is successful, this function will return a struct with on-off settings for each motor. It will then turn on (i.e. rotate at a safe speed) the appropriate motors according to the returned struct immediately after updatePid() returns. If all tasks have been correctly scheduled, the motors will spin in a recognizable pattern; otherwise the motors will not spin.

Last, but not least, the firmware additionally flash the green LED at 1 Hz and the red LED at 0.5 Hz.

Here is the video of the running program on Crazyflie.
https://www.youtube.com/watch?v=ccPVkRPeqDo