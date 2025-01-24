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

if (robot.clawServo.getPosition() >= 1.05) {
    isClawOpen = false;
}
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

        // When the button "X" is held on the Operator's Controller, then set the slider to it's maximum length, and hold until this is released.
//        if (!operator.x) {
            sliderPosition = sliderPosition + 10.0 * opRightStickY;
//        } else {
//            sliderPosition = robot.MAX_SAFE_SLIDER_TICKS;
//        }

        sliderPosition = setSliderAndReturnConstraint(sliderPosition);

        telemetry.addData("Lift","%.1f", opLeftStickY);

        // let the next frame know if the toggle was pressed
        wasClawTogglePressed = isClawTogglePressed;
    }
}

