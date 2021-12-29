package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;

@Config
@Autonomous(name = "autonomousDrive")
public class AutonomousDrive extends LinearOpMode {
    HWDriveTrain hwDriveTrain;

    BNO055IMU imu;

    Orientation angles;

    VoltageSensor voltageSensor;

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;



    public static double TURNING_Kp = 0.013;
    public static double TURNING_Ki = 0.000002;

    public static double MOVING_Kp = 0.0018;

    public static double HOLD_HEADING_Kp = 0.01;


    // Distance / Angles for autonomous

    public static double MOVE_OFF_WALL = 4;
    public static double TURN_TOWARDS_C = 90;
    public static double MOVE_TOWARDS_WALL = -50;



    @Override
    public void runOpMode() {

        hwDriveTrain = new HWDriveTrain();

        hwDriveTrain.init(this.hardwareMap);

        //voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        hwDriveTrain.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwDriveTrain.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        hwDriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                hwDriveTrain.leftFront.getCurrentPosition(),
                hwDriveTrain.rightFront.getCurrentPosition(),
                hwDriveTrain.leftBack.getCurrentPosition(),
                hwDriveTrain.rightBack.getCurrentPosition());

        telemetry.update();

        waitForStart();

        // Drive off the wall
        encoderDrive(4);
        // Turn left so the back of the robot faces the carousel
        turnToPosition(90);
        // Drive backwards to the carousel
        encoderDrive(-50);




    }

    public void turnToPosition(double target) {

        double current_angle = getHeading();

        target = current_angle + target;

        double error = target - current_angle;

        double integral = 0;

        double last_loop_time = System.currentTimeMillis();

        while( Math.abs(error) > 1 && opModeIsActive()) {
            current_angle = getHeading();
            error = target - current_angle;

            double current_loop_time = System.currentTimeMillis();

            double time_elapsed = current_loop_time - last_loop_time;

            integral += error * time_elapsed;

            double power = error * TURNING_Kp + integral * TURNING_Ki;

            hwDriveTrain.leftBack.setPower(power);
            hwDriveTrain.leftFront.setPower(power);
            hwDriveTrain.rightBack.setPower(-power);
            hwDriveTrain.rightFront.setPower(-power);

            last_loop_time = current_loop_time;

            // Set a cap on the integral value so it doesn't go crazy at the start of the rotation
            if (Math.abs(integral * TURNING_Ki) > 1) {
                // Set the integral to a value that will produce a "1" for power
                integral = 1 / TURNING_Ki;
            }


            telemetry.addData("Heading: ", current_angle);
            telemetry.addData("Error: ", error);
            telemetry.addData("Power: ", power);
            telemetry.addData("Integral: ", integral);
            telemetry.addData("Integral * Ki: ", integral * TURNING_Ki);
            telemetry.addData("Time Elapsed: ", time_elapsed);
            telemetry.update();

            idle();
        }

        hwDriveTrain.leftBack.setPower(0);
        hwDriveTrain.leftFront.setPower(0);
        hwDriveTrain.rightBack.setPower(0);
        hwDriveTrain.rightFront.setPower(0);

        telemetry.addData("Finished Turning", "Printing final values");
        telemetry.addData("Heading: ", current_angle);
        telemetry.addData("Error: ", error);
        telemetry.addData("Integral: ", integral);
        telemetry.update();
    }


    public void encoderDrive(double inches) {

        double target_position;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target_position = getAveragePosition() - (inches * COUNTS_PER_INCH);

            double distanceError = target_position - getAveragePosition();

            double target_direction = getHeading();


            while (opModeIsActive() && Math.abs(distanceError) > 25) {

                distanceError = target_position - getAveragePosition();
                double base_power = distanceError * MOVING_Kp;

                // Make power adjustments based on angle drift
                double heading_error = target_direction - getHeading();
                double rotation_power = heading_error * HOLD_HEADING_Kp;

                hwDriveTrain.leftBack.setPower(base_power + rotation_power);
                hwDriveTrain.leftFront.setPower(base_power + rotation_power);
                hwDriveTrain.rightBack.setPower(base_power - rotation_power);
                hwDriveTrain.rightFront.setPower(base_power - rotation_power);


                telemetry.addData("Error:", distanceError);
                telemetry.addData("base_power:", base_power);
                telemetry.update();
            }

            hwDriveTrain.leftBack.setPower(0);
            hwDriveTrain.leftFront.setPower(0);
            hwDriveTrain.rightBack.setPower(0);
            hwDriveTrain.rightFront.setPower(0);

            //  sleep(250);   // optional pause after each move
        }
    }


    private double getAveragePosition() {
        return (hwDriveTrain.leftFront.getCurrentPosition() + hwDriveTrain.rightFront.getCurrentPosition() + hwDriveTrain.leftBack.getCurrentPosition() + hwDriveTrain.leftFront.getCurrentPosition()) / 4.0;
    }


  public double getHeading() {
      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
  }


}
