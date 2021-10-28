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

        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        rightFront.setPower(drive + turn);
        rightBack.setPower(drive + turn);
        leftFront.setPower(drive - turn);
        leftBack.setPower(drive - turn);

    }
}
