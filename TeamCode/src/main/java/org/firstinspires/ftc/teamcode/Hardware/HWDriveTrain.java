package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class HWDriveTrain {
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public  double slowDown = 1;

    Telemetry telemetry;

    HardwareMap hwMap;

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 2.0;
    static final double WHEEL_DIAMETER_IN_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_IN_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;




    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;

        this.telemetry = telemetry;

        leftFront = (DcMotorEx) hwMap.get(DcMotor.class, "fl");
        rightFront = (DcMotorEx) hwMap.get(DcMotor.class, "fr");
        leftBack = (DcMotorEx) hwMap.get(DcMotor.class, "bl");
        rightBack = (DcMotorEx) hwMap.get(DcMotor.class, "br");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void teleOpMove(double y1, double x1, double x2, boolean isSlowdown) {
        double headingPower = 0;
        double headingAngle = 0;
        double turn = x2 * 0.5;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Convert cartesian coordinates to polar coordinates
        headingPower = Math.hypot(x1, -y1);
        headingAngle = Math.atan2(x1, -y1)  + Math.PI/4;

        slowDown = isSlowdown ? 0.5 : 1;

        double leftFrontPower = (headingPower * Math.cos(headingAngle) - turn);
        double rightFrontPower = (headingPower * Math.sin(headingAngle) + turn);
        double leftBackPower = (headingPower * Math.sin(headingAngle) - turn);
        double rightBackPower = (headingPower * Math.cos(headingAngle) + turn);

        //double maxPowerRaw = Math.max(Math.abs(leftFrontPowerRaw), Math.max(Math.abs(rightFrontPowerRaw), Math.max(Math.abs(leftBackPowerRaw), Math.abs(rightBackPowerRaw))));

        /*

        if (maxPowerRaw > 1) {
            leftFrontPowerScaled = leftFrontPowerRaw / maxPowerRaw;
            rightFrontPowerScaled = rightFrontPowerRaw / maxPowerRaw;
            leftBackPowerScaled = leftBackPowerRaw / maxPowerRaw;
            rightBackPowerScaled = rightBackPowerRaw / maxPowerRaw;
        } else {
            leftFrontPowerScaled = leftFrontPowerRaw;
            rightFrontPowerScaled = rightFrontPowerRaw;
            leftBackPowerScaled = leftBackPowerRaw;
            rightBackPowerScaled = rightBackPowerRaw;
        }

         */


        leftFront.setPower(leftFrontPower * slowDown);
        rightFront.setPower(rightFrontPower * slowDown);
        leftBack.setPower(leftBackPower * slowDown);
        rightBack.setPower(rightBackPower * slowDown);

        ArrayList<Double> velocities = new ArrayList<>();
        velocities.add(((DcMotorEx) leftFront).getVelocity());
        velocities.add(((DcMotorEx) leftBack).getVelocity());
        velocities.add(((DcMotorEx) rightFront).getVelocity());
        velocities.add(((DcMotorEx) rightBack).getVelocity());

        telemetry.addData("Wheel Velocities: ", velocities.toString());
        telemetry.update();

    }


    public void resetEncoders() {

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

