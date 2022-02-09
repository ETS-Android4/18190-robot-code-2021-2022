package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedRightDelay extends RedRightBase {

    @Override
    public void runOpMode() throws InterruptedException {
        robotWait(5000);
        redAutonomous();
    }
}
