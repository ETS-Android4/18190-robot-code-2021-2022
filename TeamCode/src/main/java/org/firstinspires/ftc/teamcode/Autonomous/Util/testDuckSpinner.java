package org.firstinspires.ftc.teamcode.Autonomous.Util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;

@Autonomous
public class testDuckSpinner extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        scoreDuck();

    }
}
