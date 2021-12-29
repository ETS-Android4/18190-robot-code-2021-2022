package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.TurningPIDConstants;


@Config
@Autonomous(name = "autonomousDrive")
public class AutonomousDrive extends LinearOpMode {
    HWDriveTrain hwDriveTrain;

    BNO055IMU imu;

    Orientation angles;

    VoltageSensor voltageSensor;

    public ElapsedTime runtime = new ElapsedTime();

    // Save starting voltage
    //double starting_voltage;


    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBuilda 5203 312 rpm
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //Define movement constants here so we can change them dynamically in FTC Dashboard
    public static double MOVE_OFF_WALL = 4;
    public static double TURN_TOWARDS_C = 90;
    public static double MOVE_TOWARDS_WALL = -43;

    // Time to wait between runs
    public static double WAIT_TIME = 2000;

    private double leftFrontPos = 0;
    private double rightFrontPos = 0;
    private double leftBackPos = 0;
    private double rightBackPos = 0;

    // Voltage Modifier
    // private double LOW_VOLTAGE = 12.8;
    // private double VOLTAGE_SLOPE_DIVISOR = 6;


    @Override
    public void runOpMode() {

        hwDriveTrain = new HWDriveTrain();

        hwDriveTrain.init(this.hardwareMap, telemetry);

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

        // Save the initial voltage
        //starting_voltage = hwDriveTrain.getBatteryVoltage();

        // Drive off the wall
        encoderDrive(MOVE_OFF_WALL);
        // Turn left so the back of the robot faces the carousel
        turnToPosition(TURN_TOWARDS_C);
        // Drive backwards to the carousel
        encoderDrive(MOVE_TOWARDS_WALL);

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

            double power = error * TurningPIDConstants.TURNING_KP + integral * TurningPIDConstants.TURNING_KI;

            hwDriveTrain.leftBack.setPower(power);
            hwDriveTrain.leftFront.setPower(power);
            hwDriveTrain.rightBack.setPower(-power);
            hwDriveTrain.rightFront.setPower(-power);

            last_loop_time = current_loop_time;

            // Set a cap on the integral value so it doesn't go crazy at the start of the rotation
            if (Math.abs(integral * TurningPIDConstants.TURNING_KI) > 1) {
                // Set the integral to a value that will produce a "1" for power
                integral = 1 / TurningPIDConstants.TURNING_KI;
            }


            telemetry.addData("Heading: ", current_angle);
            telemetry.addData("Error: ", error);
            telemetry.addData("Power: ", power);
            telemetry.addData("Integral: ", integral);
            telemetry.addData("Integral * Ki: ", integral * TurningPIDConstants.TURNING_KI);
            telemetry.addData("Time Elapsed: ", time_elapsed);
            telemetry.update();

            idle();
        }

        hwDriveTrain.leftBack.setPower(0);
        hwDriveTrain.leftFront.setPower(0);
        hwDriveTrain.rightBack.setPower(0);
        hwDriveTrain.rightFront.setPower(0);

        // Idle for 5 seconds before continuing
        double end_time = System.currentTimeMillis() + WAIT_TIME;
        while(opModeIsActive() && System.currentTimeMillis() < end_time) {
            telemetry.addData("Finished Turning", "Printing final values");
            telemetry.addData("Heading: ", current_angle);
            telemetry.addData("Error: ", error);
            telemetry.addData("Integral: ", integral);
            telemetry.update();
        }
    }


    public void encoderDrive(double inches) {

        double target_position;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target_position = getAveragePosition() - (inches * COUNTS_PER_INCH);

            double distance_error = target_position - getAveragePosition();

            double target_direction = getHeading();

            double distance_integral = 0;

            double previous_time = System.currentTimeMillis();

            double heading_error = 0;
            double base_power = 0;

            // Compute a modifier to compensate for battery voltage
            /*double voltage_modifier;
            if (starting_voltage > LOW_VOLTAGE) {
                voltage_modifier = ((starting_voltage - LOW_VOLTAGE) / VOLTAGE_SLOPE_DIVISOR) + 1;
            }
            else {
                voltage_modifier = 1;
            }
            */


            while (opModeIsActive() && Math.abs(distance_error) > 25) {

                distance_error = target_position - getAveragePosition();

                double current_time = System.currentTimeMillis();
                double dt = current_time - previous_time;
                // Add distance error to integral if we're close to the target
                if (distance_error < COUNTS_PER_INCH * MovingPIDConstants.MOVING_INTEGRAL_DISTANCE) {
                    distance_integral += distance_error * dt;
                    telemetry.addData("Distance Integral:", distance_integral);
                }

                // Reset the previous time
                previous_time = current_time;

                base_power = distance_error * MovingPIDConstants.MOVING_KP + distance_integral * MovingPIDConstants.MOVING_KI;

                // Make power adjustments based on angle drift
                heading_error = target_direction - getHeading();
                double rotation_power = heading_error * MovingPIDConstants.HOLD_HEADING_KP;



                hwDriveTrain.leftBack.setPower(base_power + rotation_power);
                hwDriveTrain.leftFront.setPower(base_power + rotation_power);
                hwDriveTrain.rightBack.setPower(base_power - rotation_power);
                hwDriveTrain.rightFront.setPower(base_power - rotation_power);

                //telemetry.addData("Voltage: ", starting_voltage);

                telemetry.addData("Left Front Encoder: ", leftFrontPos);
                telemetry.addData("Right Front Encoder: ", rightFrontPos);
                telemetry.addData("Left Back Encoder: ", leftBackPos);
                telemetry.addData("Right Back Encoder: ", rightBackPos);

                telemetry.addData("Distance Error:", distance_error);
                telemetry.addData("Heading Error:", heading_error);
                telemetry.addData("base_power:", base_power);
                telemetry.update();
            }

            hwDriveTrain.leftBack.setPower(0);
            hwDriveTrain.leftFront.setPower(0);
            hwDriveTrain.rightBack.setPower(0);
            hwDriveTrain.rightFront.setPower(0);

            hwDriveTrain.resetEncoders();

            //  sleep(250);   // optional pause after each move

            // Idle for 5 seconds before continuing
            double end_time = System.currentTimeMillis() + WAIT_TIME;
            while(opModeIsActive() && System.currentTimeMillis() < end_time) {
                telemetry.addData("Finished Movement", "");
                telemetry.addData("Distance Error:", distance_error);
                telemetry.addData("Distance Integral:", distance_integral);
                telemetry.addData("Heading Error:", heading_error);
                telemetry.addData("base_power:", base_power);
                telemetry.update();
            }
        }
    }

    private double getAveragePosition() {
        return getAveragePosition(false);
    }

    private double getAveragePosition(boolean debug) {
        double leftFrontPos = hwDriveTrain.leftFront.getCurrentPosition();
        double rightFrontPos = hwDriveTrain.rightFront.getCurrentPosition();
        double leftBackPos = hwDriveTrain.leftBack.getCurrentPosition();
        double rightBackPos = hwDriveTrain.rightBack.getCurrentPosition();

        if (debug) {
            telemetry.addData("Left Front Encoder: ", leftFrontPos);
            telemetry.addData("Right Front Encoder: ", rightFrontPos);
            telemetry.addData("Left Back Encoder: ", leftBackPos);
            telemetry.addData("Right Back Encoder: ", rightBackPos);
        }

        return (leftFrontPos + rightFrontPos + leftBackPos + rightBackPos) / 4.0;
    }

    public double getHeading() {
      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}
