# UC Berkeley EE222/ME237 Nonlinear Systems Ball and Beam Project

*Jinsheng Li, Miranda Xie, Leo Huang, and Dylan Lee*

Code and documentation for control of a nonlinear ball and beam system.

The report for part 1 (simulation) is here: [`EE222_project_report_part1.pdf`](EE222_project_report_part1.pdf) and the report for part 2 (hardware testing) is here: [`EE222_project_report_part2.pdf`](EE222_project_report_part2.pdf). Presented here is a brief summary of results.

## Observer and Controllers Used

Our group used an EKF observer. The first controller developed was a PID-Wrapped LQR controller, and the second was an Approximate Feedback Linearization controller. To switch between the two, comment/uncomment lines 41-42 in `studentControllerInterface.m`.

## Part 1: Simulation Results Summary

We summarize results of the controllers using:
* a tester trajectory which swept various amplitudes and periods of both square and sine waves, initializing ball posiiton at -19cm and servo angle at 0 degrees.
* pure sine waves with an amplitude of 7.5cm and period of 8s, initializing ball position at 0cm and servo angle at -55 degrees.
* pure square waves with an amplitude of 5cm and period of 8s, initializing ball position at 0cm and servo angle at -55 degrees.

The results are tabulated below.

| Controller | PID-LQR | PID-LQR | PID-LQR | Feedback Lin. | Feedback Lin. | Feedback Lin. |
| ---------- | ------- | ------- | ------- | ------------- | ------------- | ------------- |
| Trajectory | Tester  | Sine    | Square  | Tester        | Sine          | Square        |
||||||||
| Tracking Cost | 4.30 | 0.30    | 1.91    | 5.01          | 0.73          | 3.08          |
| Energy Cost   | 1.21 | 0.39    | 0.80    | 1.34          | 0.63          | 0.72          |
| Total Cost    | 5.51 | 0.69    | 2.71    | 6.35          | 1.35          | 3.80          |

In general, the PID-Wrapped LQR controller seems to perform better for square waves, but the feedback linearization controller tends to perform better with sine waves. The overall performance of the PID-LQR controller seems to be slightly better but this may be due to the tester trajectory including more aggressive square waves than sine waves.

Animations of the simulated controllers can be found in the ['/pt1_media'](/pt1_media) folder.

## Part 2: Hardware Testing Results Summary

We tested the developed controllers on hardware after further observation-driven tuning and modifications (see report for full details). We summarize the results of the controllers using:
* an evaluation trajectory provided to us, initializing ball posiiton at 0cm and servo angle at -55 degrees.
* pure sine waves with an amplitude of 7.5cm and period of 8s, initializing ball position at 0cm and servo angle at -55 degrees.
* pure square waves with an amplitude of 5cm and period of 8s, initializing ball position at 0cm and servo angle at -55 degrees.

The results are tabulated below.

| Controller | PID-LQR | PID-LQR | PID-LQR | Feedback Lin. | Feedback Lin. | Feedback Lin. |
| ---------- | ------- | ------- | ------- | ------------- | ------------- | ------------- |
| Trajectory | Evaluation | Sine    | Square  | Evaluation | Sine          | Square        |
||||||||
| Tracking Cost | 0.24 | 0.81    | 4.05    | 0.65          | 1.44          | 5.25          |
| Energy Cost   | 1.08 | 1.15    | 1.95    | 1.51          | 1.22          | 1.84          |
| Total Cost    | 1.36 | 1.96    | 6.00    | 2.16          | 2.65          | 7.08          |

Qualitatively, the results a similar to the simulation, with the PID-LQR controller generally outperforming the FL controller (although the FL controller can sometimes achieve similar tracking with less energy usage). However, we note that the scores seen on hardware are generally higher than in simulation, suggesting model-plant mismatch in aspects such as sensor noise, friction dynamics, and controller delays. We discuss these in detail in our report. 

### Visualizations

Below is a video of our PID-LQR controller running the evaluation trajectory.

https://github.com/user-attachments/assets/c7ad362c-3898-4642-ad77-d17687e36908

More detailed media can be found in the ['/sim_media'](/sim_media) and ['/hardware_media'](/hardware_media) folders.
