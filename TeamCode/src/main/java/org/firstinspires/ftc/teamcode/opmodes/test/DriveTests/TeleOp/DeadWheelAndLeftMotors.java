/* Copyright (c) 2021 FIRST. All rights reserved.
 * (License and header text truncated for brevity)
 */

package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.purepursuit.localization.DeadWheel;

//@TeleOp(name="Controlled Left Motors with DeadWheel", group="Linear OpMode")
//@Disabled
public class DeadWheelAndLeftMotors extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorImplEx leftFrontDrive = null;  // Use DcMotorImplEx for encoder support
    private DcMotorImplEx leftBackDrive = null;

    // Declare DeadWheel instance
    private DeadWheel leftFrontDeadWheel = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables for the left motors (use DcMotorImplEx)
        leftFrontDrive = hardwareMap.get(DcMotorImplEx.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotorImplEx.class, "lb");

        // Initialize the DeadWheel for the front left motor
        leftFrontDeadWheel = new DeadWheel(leftFrontDrive);

        // Set both left motors to move in the same direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the match ends (driver presses STOP)
        while (opModeIsActive()) {
            // Get the joystick input for forward/backward movement (left stick Y-axis)
            double motorPower = -gamepad1.left_stick_y;  // Joystick forward is negative

            // Set both left motors to the same power
            leftFrontDrive.setPower(motorPower);
            leftBackDrive.setPower(motorPower);

            // Display runtime and motor power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "Left Motors: %4.2f", motorPower);

            // Use the DeadWheel's debug method to show position and velocity on telemetry
            leftFrontDeadWheel.debug(telemetry);

            // Update telemetry data on the Driver Hub
            telemetry.update();
        }
    }
}
