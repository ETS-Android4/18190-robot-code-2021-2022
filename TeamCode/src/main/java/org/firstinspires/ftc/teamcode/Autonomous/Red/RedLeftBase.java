package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;



@Config
public abstract class RedLeftBase extends AutonomousBase {
    public static double DRIVE_TOWARDS_HUB_DISTANCE = 1;
    public static double TURN_TO_WALL_DEGREES = 130;
    public static double DRIVE_TO_WALL_DISTANCE = 23;
    public static double STRAFE_TO_DUCK_DISTANCE = -7;
    public static double STRAFE_TO_SQUARE = 7;

    public static double STRAFE_TIMEOUT = 5000;

    public void redAutonomous() {
        initializeHardware();

        waitForStart();

        // Move off the wall
        encoderDrive(-15);

        // Turn left 45 degrees
        turnToPosition(-40);

        encoderDrive(DRIVE_TOWARDS_HUB_DISTANCE);

        // Deploy the arm
        armToTopGoal();

        // Move the collector to score
        score();

        armToDrivingPosition();

        turnToPosition(TURN_TO_WALL_DEGREES);

        encoderDrive(-DRIVE_TO_WALL_DISTANCE);

        encoderDrive(STRAFE_TO_DUCK_DISTANCE, true, STRAFE_TIMEOUT);

        scoreDuck(true);

        encoderDrive(STRAFE_TO_SQUARE, true);

        armToCollectionPosition();
    }
}
