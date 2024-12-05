package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TeleOp2024 extends DriveMethods {
    boolean wasClawTogglePressed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        Gamepad driver = gamepad1;
        Gamepad operator = gamepad2;

        boolean isClawTogglePressed = operator.b;
        if (isClawTogglePressed && !wasClawTogglePressed) {
            if (robot.clawServo.getPosition() == robot.CLAW_OPEN) {
                robot.clawServo.setPosition(robot.CLAW_CLOSED);
            } else {
                robot.clawServo.setPosition(robot.CLAW_OPEN);
            }
        }


        double driveLeftStickY = -driver.left_stick_y;
        double driveRightStickY = driver.right_stick_y;
        double driveLeftStickX = driver.left_stick_x;
        double driveRightStickX = driver.right_stick_x;

        double opLeftStickY = -operator.left_stick_y;
        double opRightStickY = operator.right_stick_y;
        double opLeftStickX = operator.left_stick_x;
        double opRightStickX = operator.right_stick_x;

        omniDrive(driveLeftStickY, driveLeftStickX, driveRightStickX);
        telemetry.addData("Axial","%.1f", driveLeftStickY);
        telemetry.addData("Lateral","%.1f", driveLeftStickX);
        telemetry.addData("Yaw","%.1f", driveRightStickX);

        robot.wormGear.setPower(opLeftStickY);
        robot.sliderMotor.setPower(0.75 * opRightStickY);

        telemetry.addData("Lift","%.1f", opLeftStickY);
//Lift means wormrote variable, which refers to the rotation of the worm gear

        // let the next frame know if the toggle was pressed
        wasClawTogglePressed = isClawTogglePressed;
    }
}
