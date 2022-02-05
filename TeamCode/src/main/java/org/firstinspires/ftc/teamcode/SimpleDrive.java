package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;

@TeleOp(name="Simple Drive", group="Opmode")
public class SimpleDrive extends OpMode {
    HWDriveTrain hwDriveTrain;

    @Override
    public void init() {
        hwDriveTrain = new HWDriveTrain();
        hwDriveTrain.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        hwDriveTrain.teleOpMove(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);

        if (gamepad1.left_trigger > 0) {
            hwDriveTrain.duckMotor.setPower(-gamepad1.left_trigger);
        }
        else {
            hwDriveTrain.duckMotor.setPower(gamepad1.right_trigger);
        }

        // Arm Control
        if (Math.abs(gamepad2.left_stick_y) < 0.05) {
            if (gamepad2.dpad_up) {
                hwDriveTrain.armToTopGoal();
            }
            if (gamepad2.dpad_left) {
                hwDriveTrain.armToMiddleGoal();
            }
            if (gamepad2.dpad_down) {
                hwDriveTrain.armToBottomGoal();
            }
            if (gamepad2.left_bumper) {
                hwDriveTrain.armToCollectionPosition();
            }
            if (gamepad2.right_bumper) {
                hwDriveTrain.armToDrivingPosition();
            }
            hwDriveTrain.setArmPower();
        }
        else {
            hwDriveTrain.armTargetPosition = hwDriveTrain.armMotor.getCurrentPosition();
            double armPower = gamepad2.left_stick_y * 0.25;
            if (Math.abs(armPower) < 0.1) {
                armPower = 0;
            }
            hwDriveTrain.armMotor.setPower(armPower);
        }

        /*
        double armPower = gamepad2.left_stick_y * 0.15;
        if (Math.abs(armPower) < 0.1) {
            armPower = 0;
        }
        hwDriveTrain.armMotor.setPower(armPower);
        */


        double driveExp = 2;
        double maxSpeed = 0.5;
        double collectorPower = maxSpeed * Math.pow(Math.abs(gamepad2.right_stick_y), driveExp) / Math.pow(1.2, driveExp);
        // Collection
        if (gamepad2.right_stick_y < 0) {
            collectorPower *= -1;
        }
        else if (gamepad2.right_stick_y > 0) {
            collectorPower *= 0.5;
        }
        else {
            collectorPower = 0;
        }



        collectorPower = Range.clip(collectorPower, -1, 1);

        hwDriveTrain.collector.setPower(collectorPower);

        telemetry.addData("collector", collectorPower);
        telemetry.update();



    }
}
