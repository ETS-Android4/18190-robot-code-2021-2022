package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueLeftDelay extends BlueLeftBase {

    @Override
    public void runOpMode() throws InterruptedException {
        robotWait(5000);
        blueAutonomous();
    }
}
