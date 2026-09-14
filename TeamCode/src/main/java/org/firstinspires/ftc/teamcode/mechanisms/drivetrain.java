package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This is not an OpMode itself, that can be run on the Driver Hub. Instead, this is a mechanism file.
 * These mechanism files have sets of motors, sensors, servos, and other hardware.
 * In addition, they also have specific software methods that can be accessed through this file.
 * That way, if there are any errors with the subsystem, they can be fixed in one place.
 * Having these files means we only need to write less code.
 * In the robot file you're implementing this subsystem in, you would write this:
 * drivetrain "name_of_drivetrain" = new drivetrain(this);
 */

public class drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null; // gain access to methods in the calling OpMode.

    // Define motor objects. We make them private so they can't be accessed outside of this file.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // This declares the IMU needed to get the current direction the robot is facing
    // We'll use the IMU when we drive field-relative. Normally, we would drive robot-relative.

    IMU imu;

    // Define Drive constants. These will be accessed elsewhere, so we can make these public.

    public static final double precision_speed = 0.5;
    public static final double super_precision_speed = 0.333;

    public drivetrain (LinearOpMode opMode) { myOpMode = opMode; }

    // This code intializes our motor
    public void init() {
        frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
        myOpMode.telemetry.update();
    }

    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftMotor.setPower(frontLeftWheel);
        frontRightMotor.setPower(frontRightWheel);
        backLeftMotor.setPower(backLeftWheel);
        backRightMotor.setPower(backRightWheel);
    }

    public void stopDriving() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void resetYaw() {
        imu.resetYaw();
    }

    // Code largely copied from the RobotTeleopMecanumFieldRelativeDrive.java file
    // The following method was credited to FTC 16072, so I also credit this code to them.
    // I've only made minor changes to it.
    public void driveRobotRelative(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate.
        double FrontLeftPower = forward + right + rotate;
        double FrontRightPower = forward - right - rotate;
        double BackRightPower = forward + right - rotate;
        double BackLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0; // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped.
        maxPower = Math.max(maxPower, Math.abs(FrontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRightPower));
        maxPower = Math.max(maxPower, Math.abs(BackRightPower));
        maxPower = Math.max(maxPower, Math.abs(BackLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftMotor.setPower(maxSpeed * (FrontLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (FrontRightPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (BackLeftPower / maxPower));
        backRightMotor.setPower(maxSpeed * (BackRightPower / maxPower));
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        driveRobotRelative(newForward, newRight, rotate);
    }
}
