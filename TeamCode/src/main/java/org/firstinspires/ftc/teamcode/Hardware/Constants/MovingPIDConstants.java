package org.firstinspires.ftc.teamcode.Hardware.Constants;

public interface MovingPIDConstants {
    double MOVING_KP = 0.0018;
    double MOVING_KI = 0.00002;
    double MOVING_KD = 0;

    double HOLD_HEADING_KP = 0.01;
    double HOLD_HEADING_KI = 0;
    double HOLD_HEADING_KD = 0;
    // Distance (in inches) from target to start using integral term
    double MOVING_INTEGRAL_DISTANCE = 2;
}
