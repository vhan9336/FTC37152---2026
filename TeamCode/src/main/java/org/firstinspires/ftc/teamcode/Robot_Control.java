package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.drivetrain;

@TeleOp
public class Robot_Control extends LinearOpMode {

    // Creating a drivetrain object to be used to access robot hardware.
    // Prefix any hardware functions with "drivetrain." to access this class.
    drivetrain drivetrain = new drivetrain(this);

    @Override
    public void runOpMode() {
        drivetrain.init();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Drive", "Left Stick Y");
            telemetry.addData("Drive Power", "%.2f", -gamepad1.left_stick_y);
            telemetry.addData("Strafe", "Left Stick X");
            telemetry.addData("Strafe Power", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("Turn", "Right Stick X");
            telemetry.addData("Turn Power", "%.2f", gamepad1.right_stick_x);
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");
            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                drivetrain.resetYaw();
            }
            // If you press the left bumper, you drive from the point of view of the field
            // Otherwise, you drive from the point of view of the robot
            // (much like driving an RC vehicle)
            if (gamepad1.left_bumper) {
                drivetrain.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            else {
                drivetrain.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
    }
}
