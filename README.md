# UC Berkeley EE222/ME237 Nonlinear Systems Ball and Beam Project

*Jinsheng Li, Miranda Xie, Leo Huang, and Dylan Lee*

The full project report is here: [`EE222_project_report_part1.pdf`](EE222_project_report_part1.pdf). Presented here is a brief summary of results.

## Observer and Controllers Used

Our group used an EKF observer. The first controller developed was a PID-Wrapped LQR controller, and the second was an Approximate Feedback Linearization controller. To switch between the two, comment/uncomment lines 38-39 in `studentControllerInterface.m`.

## Simulation Results Summary

We summarize results of the controllers using:
* a tester trajectory which swept various amplitudes and periods of both square and sine waves, initializing ball posiiton at -19cm
* pure sine waves with an amplitude of 7.5cm and period of 8s, initializing ball position at 0cm
* pure square waves with an amplitude of 5cm and period of 8s, initializing ball position at 0cm.

The results are tabulated below.

| Controller | PID-LQR | PID-LQR | PID-LQR | Feedback Lin. | Feedback Lin. | Feedback Lin. |
| ---------- | ------- | ------- | ------- | ------------- | ------------- | ------------- |
| Trajectory | Tester  | Sine    | Square  | Tester        | Sine          | Square        |
||||||||
| Tracking Cost | 4.30 | 0.01    | 3.10    | 5.01          | 0.01          | 4.10          |
| Energy Cost   | 1.21 | 0.03    | 1.34    | 1.34          | 0.02          | 0.57          |
| Total Cost    | 5.51 | 0.04    | 4.44    | 6.35          | 0.03          | 4.67          |

In general, the PID-Wrapped LQR controller seems to perform better for square waves, but the feedback linearization controller tends to perform better with sine waves. The overall performance of the PID-LQR controller seems to be slightly better but this may be due to the tester trajectory including more aggressive square waves than sine waves.

## Animations

Animations are in the ['/media'](/media) folder.
