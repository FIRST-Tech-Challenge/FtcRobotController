package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TeleOp2024 extends DriveMethods {
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        Gamepad driver = gamepad1;
        Gamepad operator = gamepad2;

        double leftStickY = -driver.left_stick_y;
        double rightStickY = driver.right_stick_y;
        double leftStickX = driver.left_stick_x;
        double rightStickX = driver.right_stick_x;

        omniDrive(leftStickY, leftStickX, rightStickX);

    }
}
