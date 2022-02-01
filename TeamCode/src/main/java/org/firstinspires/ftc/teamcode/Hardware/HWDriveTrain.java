package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class HWDriveTrain {
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public DcMotorEx duckMotor;

    public DcMotorEx armMotor;

    public CRServo collector;

    public  double slowDown = 1;

    Telemetry telemetry;

    HardwareMap hwMap;



    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;

        this.telemetry = telemetry;

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
}

