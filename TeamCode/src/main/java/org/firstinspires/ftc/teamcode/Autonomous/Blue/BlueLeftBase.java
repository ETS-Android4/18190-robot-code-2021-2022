package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;

public abstract class BlueLeftBase extends AutonomousBase {
    public void blueAutonomous() {
        // Hardware initialization happens in the superclass (AutonomousBase)
        initializeHardware();

        waitForStart();

        // Move off the wall
        encoderDrive(-15);

        // Turn right 45 degrees
        turnToPosition(-40);

        encoderDrive(-4);

        // Deploy the arm
        armToTopGoal();

        // Move the collector to score
        score();

        armToDrivingPosition();

        turnToPosition(-50);

        encoderDrive(48);

        armToCollectionPosition();
    }
}
