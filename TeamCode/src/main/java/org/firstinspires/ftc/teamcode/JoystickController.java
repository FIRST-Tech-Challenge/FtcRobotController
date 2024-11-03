package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

class JoystickController {
    private Gamepad gamepad;
    private MecanumDrive mecanumDrive;

    // Constructor that takes the gamepad and mecanum drive as inputs
    public JoystickController(Gamepad gamepad, MecanumDrive mecanumDrive) {
        this.gamepad = gamepad;
        this.mecanumDrive = mecanumDrive;
        System.out.println("Joystick controller initialized");
    }

    // Update method to read joystick inputs and control the robot accordingly
    public void update() {
        // Forward/backward movement is controlled by the left stick Y-axis (inverted)
        double forward = -gamepad.left_stick_y;  // Reverse because pushing stick forward gives a negative value
        // Left/right strafing is controlled by the left stick X-axis
        double strafe = gamepad.left_stick_x;
        // Rotation is controlled by the right stick X-axis
        double rotate = gamepad.right_stick_x;
        // Drive the robot based on the joystick inputs
        mecanumDrive.drive(forward, strafe, rotate);
        System.out.println(String.format("Joystick update - Forward: %2.2f, Strafe: %2.2f, Rotate: %2.2f", forward, strafe, rotate));
    }
}