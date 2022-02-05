package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;

@Autonomous
public class BlueLeftScore extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {

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
    }
}
