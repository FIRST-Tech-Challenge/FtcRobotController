package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class JoystickController {
    private Gamepad gamepad;
    private MecanumDrive mecanumDrive;

    public JoystickController(Gamepad gamepad, MecanumDrive mecanumDrive) {
        this.gamepad = gamepad;
        this.mecanumDrive = mecanumDrive;
    }

    public void update() {
        double forward = -gamepad.left_stick_y;  // Reverse because pushing stick forward is negative
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;
        mecanumDrive.drive(forward, strafe, rotate);
    }
}
