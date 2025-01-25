package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntoTheDeepTeleOp", group = "TeleOp")
public class IntoTheDeepTeleOp extends OpMode {
    private MecanumDrive mecanumDrive;
    private JoystickController joystickController;
    private Robot robot;
    private Intake intake;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean leftBumperPressed = false;

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
        intake = new Intake(hardwareMap, "intake_slide");

        // Initialize the complete robot
        robot = new Robot(mecanumDrive, joystickController);
    }

    @Override
    public void loop() {
        // Update joystick controls
        joystickController.update();
        
        // Intake controls
        if (gamepad1.dpad_up) {
            intake.forward();
        } else if (gamepad1.dpad_down) {
            intake.backward();
        } else {
            intake.stop();
        }
        
        // Existing intake controls
        if (gamepad1.dpad_right) {
            intake.out(true);
        } else if (gamepad1.dpad_left) {
            intake.in();
        } else if (gamepad1.left_bumper) {
            intake.out(false);
        }
        
        // Update telemetry
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