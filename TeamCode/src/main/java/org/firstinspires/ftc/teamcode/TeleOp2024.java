package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TeleOp2024 extends DriveMethods {
    boolean wasClawTogglePressed = false;
    double sliderPosition = robot.MIN_SLIDER_TICKS;
    boolean isClawOpen = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.sliderMotor.setPower(1);
        robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawServo.setPosition(robot.CLAW_CLOSED);


    }

    @Override
    public void loop() {
        Gamepad driver = gamepad1;
        Gamepad operator = gamepad2;

        boolean isClawTogglePressed = operator.b;
        if (isClawTogglePressed && !wasClawTogglePressed) {
            if (isClawOpen) {
                robot.clawServo.setPosition(robot.CLAW_CLOSED);
                isClawOpen = false;
            } else {
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                isClawOpen = true;
            }
        }


        double driveLeftStickY = -driver.left_stick_y;
        double driveRightStickY = driver.right_stick_y;
        double driveLeftStickX = driver.left_stick_x;
        double driveRightStickX = driver.right_stick_x;

        double opLeftStickY = -operator.left_stick_y;
        double opRightStickY = -operator.right_stick_y;
        double opLeftStickX = operator.left_stick_x;
        double opRightStickX = operator.right_stick_x;

        omniDrive(driveLeftStickY, driveLeftStickX, driveRightStickX);
        telemetry.addData("Axial","%.1f", driveLeftStickY);
        telemetry.addData("Lateral","%.1f", driveLeftStickX);
        telemetry.addData("Yaw","%.1f", driveRightStickX);

        double wormGearPower = opLeftStickY;
        telemetry.addData("Worm Gear Angle", "%.1f", robot.wormGearAngle());

        // don't allow the worm gear to go up beyond the max limit
        if (robot.wormGearAngle() > 95 && wormGearPower > 0) {
            wormGearPower = 0;
        }

//       if (robot.wormGearAngle() < robot.STARTING_ANGLE - 10 && wormGearPower < 0) {
//            wormGearPower = 0;
//        }

        robot.wormGear.setPower(wormGearPower);

        sliderPosition = sliderPosition + 10.0 * opRightStickY;
        if (sliderPosition > robot.MAX_HORIZONTAL_SLIDER_TICKS) {
            sliderPosition = robot.MAX_HORIZONTAL_SLIDER_TICKS;
        }
        if (sliderPosition < robot.MIN_SLIDER_TICKS) {
            sliderPosition = robot.MIN_SLIDER_TICKS;
        }
        robot.sliderMotor.setTargetPosition((int) sliderPosition);

        telemetry.addData("Lift","%.1f", opLeftStickY);
//Lift means wormrote variable, which refers to the rotation of the worm gear

        // let the next frame know if the toggle was pressed
        wasClawTogglePressed = isClawTogglePressed;
    }
}
