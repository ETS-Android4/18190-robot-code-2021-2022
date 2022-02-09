package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomousRotationTest extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

        turnToPosition(180);

        sleep(1000);

        turnToPosition(45);

        sleep(1000);

        turnToPosition(-355);

    }
}
