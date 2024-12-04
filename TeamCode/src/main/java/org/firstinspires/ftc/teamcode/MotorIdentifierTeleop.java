package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorIdentifierTeleop extends DriveMethods {
    int testingMotor = 0;
    double targetClawPosition = 0;
    boolean wasPressed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        boolean isPressed = gamepad1.a;
        if (isPressed && !wasPressed) {
            testingMotor = testingMotor + 1;
            if (testingMotor > 6) {
                testingMotor = 0;
            }
        }

        DcMotorEx motor;
        String name;

        if (testingMotor == 0) {
            motor = robot.leftFrontDrive;
            name = "leftFront";
        } else if (testingMotor == 1) {
            motor = robot.rightFrontDrive;
            name = "rightFront";
        } else if (testingMotor == 2) {
            motor = robot.rightBackDrive;
            name = "rightBack";
        } else if (testingMotor == 3) {
            motor = robot.leftBackDrive;
            name = "leftBack";
        } else if (testingMotor == 4) {
            motor = robot.sliderMotor;
            name = "sliderMotor";
        } else {
            name = "wormGear";
            motor = robot.wormGear;
        }

        double power = -gamepad1.left_stick_y;
        motor.setPower(power);
//spider ::::-/
        if (gamepad1.b) {
            targetClawPosition = targetClawPosition + -gamepad1.right_stick_y * .005;
            robot.clawServo.setPosition(targetClawPosition);
        }

        if (isPressed && !wasPressed) {
            telemetry.speak(name);
        }

        telemetry.addData("motor", name);
        telemetry.addData("power", "%.1f", power);
        telemetry.addData("sliderTicks", "%.1f", (double) robot.sliderMotor.getCurrentPosition());
        telemetry.addData("clawPosition", "%.3f", robot.clawServo.getPosition());

        wasPressed = isPressed;
    }
}
