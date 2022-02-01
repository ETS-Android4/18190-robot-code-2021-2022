package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        double armPower = gamepad2.left_stick_y * 0.5;
        if (Math.abs(armPower) < 0.1) {
            armPower = 0;
        }
        hwDriveTrain.armMotor.setPower(armPower);

        double collectorPower;
        // Collection
        if (gamepad2.right_stick_y > 0) {
            collectorPower = Math.log(gamepad2.right_stick_y + 1) * 3.4;
        }
        else if (gamepad2.right_stick_y < 0) {
            collectorPower = -Math.log(Math.abs(gamepad2.right_stick_y) + 1) * 3.4;
        }
        else{
            collectorPower = 0;
        }
        hwDriveTrain.collector.setPower(collectorPower);

        telemetry.addData("collector", collectorPower);
        telemetry.update();

    }
}
