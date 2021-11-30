package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Simple Drive", group="Opmode")
public class SimpleDrive extends OpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;


    @Override
    public void init() {

        rightFront = hardwareMap.get(DcMotor.class, "fr");
        leftFront = hardwareMap.get(DcMotor.class, "fl");
        rightBack = hardwareMap.get(DcMotor.class, "br");
        leftBack = hardwareMap.get(DcMotor.class, "bl");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    @Override
    public void loop() {

        double forward = Math.pow(gamepad1.right_stick_y,2);
        if (gamepad1.right_stick_y < 0) {
            forward *= -1;
        }
        double strafe = Math.pow(gamepad1.left_stick_x,2);
        if (gamepad1.left_stick_x < 0) {
            strafe *= -1;
        }
        double turn = Math.pow(gamepad1.right_stick_x,2);
        if (gamepad1.right_stick_x < 0) {
            turn *= -1;
        }

        double angle = Math.atan2(forward, -strafe) - Math.PI/4;
        double mag = Math.hypot(forward, -strafe);

        double mod = 1;

        if (gamepad1.right_bumper) {
            mod = 0.5;
        }

        double lfPower = (mag * Math.cos(angle) + turn) * mod;
        double rfPower = (mag * Math.sin(angle) - turn) * mod;
        double lbPower = (mag * Math.sin(angle) + turn) * mod;
        double rbPower = (mag * Math.cos(angle) - turn) * mod;







        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);
        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);



    }
}
