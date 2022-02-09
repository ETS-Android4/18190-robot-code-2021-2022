package org.firstinspires.ftc.teamcode.Autonomous.Red;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;

public abstract class RedLeftBase extends AutonomousBase {
    public void redAutonomous() {
        initializeHardware();

        waitForStart();

        // Move off the wall
        encoderDrive(-15);

        // Turn left 45 degrees
        turnToPosition(-40);

        encoderDrive(-4);

        // Deploy the arm
        armToTopGoal();

        // Move the collector to score
        score();

        armToDrivingPosition();

        turnToPosition(155);

        encoderDrive(-38);

        scoreDuck();

        armToCollectionPosition();
    }
}
