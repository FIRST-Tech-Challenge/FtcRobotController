package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMecanumDrive", group = "TeleOp")
public class TeleOpMecanumDrive extends OpMode {
    private MecanumDrive mecanumDrive;
    private JoystickController joystickController;
    private Robot robot;

    @Override
    public void init() {
        // Initialize hardware devices
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        // Initialize subsystems
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        joystickController = new JoystickController(gamepad1, mecanumDrive);

        // Initialize the complete robot
        robot = new Robot(mecanumDrive, joystickController);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update joystick control to drive the robot
        robot.updateJoystickControl();
        
        // Display motor powers
        telemetry.addData("Front Left Power", "%.2f", mecanumDrive.getFrontLeftPower());
        telemetry.addData("Front Right Power", "%.2f", mecanumDrive.getFrontRightPower());
        telemetry.addData("Back Left Power", "%.2f", mecanumDrive.getBackLeftPower());
        telemetry.addData("Back Right Power", "%.2f", mecanumDrive.getBackRightPower());
        telemetry.update();
    }
}

/* Example usage for both autonomous and teleop:
 * Robot myRobot = new Robot(new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor),
 *                           new JoystickController(gamepad1, new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)));
 *
 * Teleop:
 * myRobot.updateJoystickControl();
 *
 * Autonomous:
 * myRobot.drive(0.5, 0, 0); // Move forward at 50% power
 */
