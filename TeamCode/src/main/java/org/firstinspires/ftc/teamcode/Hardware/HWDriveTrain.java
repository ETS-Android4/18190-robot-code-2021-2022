package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstantsNoVelocity;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstantsWithVelocity;


public class HWDriveTrain {

    public static double ARM_KP = 0.001;

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public DcMotorEx duckMotor;

    public DcMotorEx armMotor;

    public CRServo collector;

    public  double slowDown = 1;

    public Telemetry telemetry;

    public HardwareMap hwMap;

    public double armTargetPosition;

    public double armError;

    public BNO055IMU imu;

    public Orientation angles;


    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;

        this.telemetry = telemetry;

        armTargetPosition = 0;

        leftFront = (DcMotorEx) hwMap.get(DcMotor.class, "fl");
        rightFront = (DcMotorEx) hwMap.get(DcMotor.class, "fr");
        leftBack = (DcMotorEx) hwMap.get(DcMotor.class, "bl");
        rightBack = (DcMotorEx) hwMap.get(DcMotor.class, "br");

        duckMotor = (DcMotorEx) hwMap.get(DcMotor.class, "duck");

        armMotor = (DcMotorEx) hwMap.get(DcMotor.class, "arm");


        //collector = hwMap.crservo.get("collector");
        collector = hwMap.crservo.get("collector");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        duckMotor.setPower(0);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //collector.setPower(0);
    }

    public void teleOpMove(double y1, double x1, double x2, boolean isSlowdown) {
        double turn = x2 * 0.5;

        //Convert cartesian coordinates to polar coordinates
        double headingPower = Math.hypot(x1, -y1);
        double headingAngle = Math.atan2(x1, -y1)  + Math.PI/4;

        slowDown = isSlowdown ? 0.5 : 1;

        double leftFrontPower = (headingPower * Math.cos(headingAngle) - turn);
        double rightFrontPower = (headingPower * Math.sin(headingAngle) + turn);
        double leftBackPower = (headingPower * Math.sin(headingAngle) - turn);
        double rightBackPower = (headingPower * Math.cos(headingAngle) + turn);



        leftFront.setPower(leftFrontPower * slowDown);
        rightFront.setPower(rightFrontPower * slowDown);
        leftBack.setPower(leftBackPower * slowDown);
        rightBack.setPower(rightBackPower * slowDown);

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

    public void setArmPower() {
        double armPosition = armMotor.getCurrentPosition();
        armError = armTargetPosition - armPosition;
        double armMotorPower = armError * ARM_KP;
        armMotor.setPower(armMotorPower);

        telemetry.addData("Arm Error", armError);
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Arm Power", armMotorPower);
        telemetry.addData("Arm Target Position", armTargetPosition);
    }

    public void armToTopGoal() {
        armTargetPosition = -1884;
    }

    public void armToMiddleGoal() {
        armTargetPosition = -2206;
    }

    public void armToBottomGoal() {
        armTargetPosition = -2358;
    }

    public void armToDrivingPosition() {
        armTargetPosition = -500;
    }

    public void armToCollectionPosition() {
        armTargetPosition = 0;
    }

}

