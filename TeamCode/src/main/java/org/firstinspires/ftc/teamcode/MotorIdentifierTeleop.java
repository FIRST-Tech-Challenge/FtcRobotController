package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorIdentifierTeleop extends DriveMethods {
    int testingMotor = 0;
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
            if (testingMotor > 4) {
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
        } else {
            name = "wormGear";
            motor = robot.wormGear;
        }

        double power = -gamepad1.left_stick_y;
        motor.setPower(power);

        if (isPressed && !wasPressed) {
            telemetry.speak(name);
        }

        telemetry.addData("motor", name);
        telemetry.addData("power", "%.1f", power);

        wasPressed = isPressed;
    }
}
