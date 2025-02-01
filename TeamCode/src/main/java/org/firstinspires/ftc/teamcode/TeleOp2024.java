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
        double driveRightStickY = -driver.right_stick_y;
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
        if (robot.wormGearAngle() >= 85 && wormGearPower > 0.0) {
            wormGearPower = 0.0;
        }

//       if (robot.wormGearAngle() < robot.STARTING_ANGLE - 10 && wormGearPower < 0) {
//            wormGearPower = 0;
//        }

        boolean slowMode2 = gamepad2.left_bumper;
        double SLOW_2_MODE_SPEED = .25;
        if (slowMode2) {
            robot.wormGear.setPower(SLOW_2_MODE_SPEED * wormGearPower);

        } else {
            robot.wormGear.setPower(wormGearPower);
        }

        // When the button "X" is held on the Operator's Controller, then set the slider to it's maximum length, and hold until this is released.
        // Utilize this for faster access to top basket
        // If a is clicked, then the slider is set to it's minimum.

       if (!operator.x) {
            sliderPosition = sliderPosition + 10.0 * opRightStickY;
       } else if (operator.a) {
           sliderPosition = robot.MIN_SLIDER_TICKS;
       } else if (operator.right_bumper) {
           robot.sliderMotor.setPower(0);
       } else {
           sliderPosition = robot.MAX_SAFE_SLIDER_TICKS;
        }

       // End "X" & "A" Button Code

        sliderPosition = setSliderAndReturnConstraint(sliderPosition);

       //Set the worm gear tol -7 degrees

        if (operator.right_bumper && operator.left_bumper && driver.right_bumper && driver.left_bumper) {
            robot.wormGear.setPower(-1);
            if (robot.wormGearAngle() == -7) {
                robot.wormGear.setPower(0);
            }
        }
        telemetry.addData("Lift","%.1f", opLeftStickY);

        // let the next frame know if the toggle was pressed
        wasClawTogglePressed = isClawTogglePressed;
    }
}

