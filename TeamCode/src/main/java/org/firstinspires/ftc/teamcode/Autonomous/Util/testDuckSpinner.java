package org.firstinspires.ftc.teamcode.Autonomous.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousBase;

@Autonomous
@Config
public class testDuckSpinner extends AutonomousBase {

    public static double TIME_BETWEEN_DUCKS = 1250;

    public static double NUM_DUCKS = 8;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        initializeHardware();

        for (int i = 0; i < NUM_DUCKS; i++) {
            scoreDuck();
            sleep(Double.valueOf(TIME_BETWEEN_DUCKS).longValue());
        }


    }
}
