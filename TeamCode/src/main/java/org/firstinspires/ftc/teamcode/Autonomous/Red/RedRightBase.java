package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;


public abstract class RedRightBase extends AutonomousBase {


    public void redAutonomous() {
        // Hardware initialization happens in the superclass (AutonomousBase)
        initializeHardware();

        waitForStart();

        // Move off the wall
        encoderDrive(-15);

        // Turn left 45 degrees
        turnToPosition(40);

        encoderDrive(-4);

        // Deploy the arm
        armToTopGoal();

        // Move the collector to score
        score();

        armToDrivingPosition();

        turnToPosition(50);

        encoderDrive(48);

        armToCollectionPosition();
    }

}
