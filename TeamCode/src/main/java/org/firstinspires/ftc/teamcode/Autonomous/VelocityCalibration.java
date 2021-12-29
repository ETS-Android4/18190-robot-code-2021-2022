package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;
@Autonomous
public class VelocityCalibration extends LinearOpMode {

    HWDriveTrain hwDriveTrain;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hwDriveTrain = new HWDriveTrain();

        hwDriveTrain.init(this.hardwareMap, telemetry);

        //voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        hwDriveTrain.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        hwDriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx leftFront = (DcMotorEx) hwDriveTrain.leftFront;
        DcMotorEx rightFront = (DcMotorEx) hwDriveTrain.rightFront;
        DcMotorEx leftBack = (DcMotorEx) hwDriveTrain.leftBack;
        DcMotorEx rightBack = (DcMotorEx) hwDriveTrain.rightBack;



        waitForStart();

        double currentPower = 0;
        double leftFrontVelocity = leftFront.getVelocity();
        double rightFrontVelocity = rightFront.getVelocity();
        double leftBackVelocity = leftBack.getVelocity();
        double rightBackVelocity = rightBack.getVelocity();

        double VELOCITY_THRESHOLD = 100;

        while(leftFrontVelocity <= VELOCITY_THRESHOLD && rightFrontVelocity <= VELOCITY_THRESHOLD && leftBackVelocity <= VELOCITY_THRESHOLD && rightBackVelocity <= VELOCITY_THRESHOLD && opModeIsActive()) {
            leftFrontVelocity = leftFront.getVelocity();
            rightFrontVelocity = rightFront.getVelocity();
            leftBackVelocity = leftBack.getVelocity();
            rightBackVelocity = rightBack.getVelocity();

            leftFront.setPower(currentPower);
            rightFront.setPower(currentPower);
            leftBack.setPower(currentPower);
            rightBack.setPower(currentPower);

            currentPower += 0.001;

            telemetry.addData("Current Motor Power:", currentPower);
            telemetry.update();
        }

        telemetry.addData("Final Motor Power:", currentPower);
        telemetry.update();

        while(opModeIsActive()) {
            idle();
        }

    }
}
