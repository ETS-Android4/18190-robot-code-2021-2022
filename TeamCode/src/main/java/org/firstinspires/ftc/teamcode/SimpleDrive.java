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
    }
}
