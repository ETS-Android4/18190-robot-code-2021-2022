package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstantsRegular;
import org.firstinspires.ftc.teamcode.Hardware.Constants.MovingPIDConstantsStrafe;
import org.firstinspires.ftc.teamcode.Hardware.Constants.TurningPIDConstants;
import org.firstinspires.ftc.teamcode.Hardware.HWDriveTrain;


public abstract class AutonomousBase extends LinearOpMode {


    public HWDriveTrain hwDriveTrain;

    public BNO055IMU imu;


    public Orientation angles;



    public static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBuilda 5203 312 rpm
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED = 0.5;


    public static final double DRIVE_MOTOR_RPM = 312;
    public static final double DRIVE_MOTOR_RPS = DRIVE_MOTOR_RPM / 60;

    // The real maximum speed achievable by our robot, measured in encoder counts per second
    public static final double MEASURED_MAXIMUM_TPS = 1400;
    // The theoretical max TPS of the motor
    public static final double THEORETICAL_MAX_TPS = DRIVE_MOTOR_RPS * COUNTS_PER_MOTOR_REV;

    public static final double DRIVE_MOTOR_SPEED_ADJUSTMENT = MEASURED_MAXIMUM_TPS / THEORETICAL_MAX_TPS;

    //Define movement constants here so we can change them dynamically in FTC Dashboard
    public static double MOVE_OFF_WALL = 3;
    public static double TURN_TOWARDS_C = 87;
    public static double MOVE_TOWARDS_WALL = -43;

    // Time to wait between runs
    public static double WAIT_TIME = 2000;

    // Absolute robot heading
    public double headingModifier = 0;
    public double lastHeading = 0;

    public void initializeHardware() {
        hwDriveTrain = new HWDriveTrain();

        hwDriveTrain.init(this.hardwareMap, telemetry);

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
    }

    public void robotWait(double time) {
        double start_time = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - start_time < time) {
            idle();
        }
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

            double power = error * (TurningPIDConstants.TURNING_KP / 3) + integral * TurningPIDConstants.TURNING_KI;

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

            telemetry.addData("Target Heading: ", target);
            telemetry.addData("Heading: ", current_angle);
            telemetry.addData("Error: ", error);
            telemetry.addData("Integral: ", integral);
            telemetry.update();
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
        encoderDrive(inches, false);
    }

    public void encoderDrive(double inches, boolean strafe) {
        MovingPIDConstants pid_constants;
        // Set up constants based on globals and chosen mode
        if (strafe) {
            pid_constants = new MovingPIDConstantsStrafe();
        }
        else {
            pid_constants = new MovingPIDConstantsRegular();
        }

        hwDriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwDriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            while (opModeIsActive() && Math.abs(distance_error) > 25) {

                distance_error = target_position - getAveragePosition();

                double current_time = System.currentTimeMillis();
                double dt = current_time - previous_time;
                // Add distance error to integral if we're close to the target
                if (distance_error < COUNTS_PER_INCH * pid_constants.MOVING_INTEGRAL_DISTANCE) {
                    distance_integral += distance_error * dt;
                    telemetry.addData("Distance Integral:", distance_integral);
                }

                // Reset the previous time
                previous_time = current_time;

                base_power = Range.clip(distance_error * pid_constants.MOVING_KP + distance_integral * pid_constants.MOVING_KI, -0.4, 0.4);


                // Make power adjustments based on angle drift
                heading_error = target_direction - getHeading();
                double rotation_power = heading_error * pid_constants.HOLD_HEADING_KP;

                if (strafe) {
                    hwDriveTrain.leftBack.setPower(base_power + rotation_power);
                    hwDriveTrain.leftFront.setPower(-base_power + rotation_power);
                    hwDriveTrain.rightBack.setPower(-base_power - rotation_power);
                    hwDriveTrain.rightFront.setPower(base_power - rotation_power);
                }
                else {
                    hwDriveTrain.leftBack.setPower(base_power + rotation_power);
                    hwDriveTrain.leftFront.setPower(base_power + rotation_power);
                    hwDriveTrain.rightBack.setPower(base_power - rotation_power);
                    hwDriveTrain.rightFront.setPower(base_power - rotation_power);
                }
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
                /*
                telemetry.addData("Finished Movement", "");
                telemetry.addData("Distance Error:", distance_error);
                telemetry.addData("Distance Integral:", distance_integral);
                telemetry.addData("Heading Error:", heading_error);
                telemetry.addData("base_power:", base_power);
                telemetry.update();
                 */
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

        return (leftFrontPos + rightFrontPos + leftBackPos + rightBackPos) / 4.0;
    }

    public double getHeading() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double currentReading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        double upperBound = 150;
        double lowerBound = -150;

        if (lastHeading > upperBound && currentReading < 0) {
            headingModifier += 1;
        }
        else if (lastHeading < lowerBound && currentReading > 0) {
            headingModifier -= 1;
        }

        lastHeading = currentReading;

        currentReading += headingModifier * 360;
        return currentReading;
    }

    public void armToTopGoal() {
        hwDriveTrain.armToTopGoal();
        waitForArm();
    }

    public void armToDrivingPosition() {
        hwDriveTrain.armToDrivingPosition();
        waitForArm();
    }

    public void armToCollectionPosition() {
        hwDriveTrain.armToCollectionPosition();
        waitForArm();
    }

    private void waitForArm() {
        hwDriveTrain.setArmPower();
        while (Math.abs(hwDriveTrain.armError) > 10) {
            hwDriveTrain.setArmPower();
        }
        hwDriveTrain.armMotor.setPower(0);
    }

    public void score() {
        hwDriveTrain.collector.setPower(0.2);
        robotWait(3000);
        hwDriveTrain.collector.setPower(0);
    }

    public void scoreDuck() {
        hwDriveTrain.duckMotor.setPower(-0.5);
        sleep(1000);
        hwDriveTrain.duckMotor.setPower(-1);
        sleep(2000);
        hwDriveTrain.duckMotor.setPower(0);
    }


}
