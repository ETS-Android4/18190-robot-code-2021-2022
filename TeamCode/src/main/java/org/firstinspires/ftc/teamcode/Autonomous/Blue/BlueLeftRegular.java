package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;

@Autonomous
public class BlueLeftRegular extends BlueLeftBase {

    @Override
    public void runOpMode() throws InterruptedException {
        blueAutonomous();
    }
}
