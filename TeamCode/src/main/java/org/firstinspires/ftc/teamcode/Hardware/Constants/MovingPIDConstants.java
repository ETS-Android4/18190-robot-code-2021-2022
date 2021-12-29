package org.firstinspires.ftc.teamcode.Hardware.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MovingPIDConstants {
    public static double MOVING_KP = 0.0018;
    public static double MOVING_KI = 0.00002;
    public static double MOVING_KD = 0;

    public static double HOLD_HEADING_KP = 0.01;
    public static double HOLD_HEADING_KI = 0;
    public static double HOLD_HEADING_KD = 0;
    // Distance from target to start using integral term
    public static double MOVING_INTEGRAL_DISTANCE = 2;
}
