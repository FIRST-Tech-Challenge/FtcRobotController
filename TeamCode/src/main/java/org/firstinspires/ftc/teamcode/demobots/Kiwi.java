package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

@TeleOp(name = "Kiwi", group = "DemoBot")
public class Kiwi extends OpMode {

    private DcMotor motorOne;
    private DcMotor motorTwo;
    private DcMotor motorThree;
    private boolean pressed = false;
    private double speed = 2;

    @Override
    public void init() {
        motorOne = hardwareMap.dcMotor.get("one");
        motorTwo = hardwareMap.dcMotor.get("two");
        motorThree = hardwareMap.dcMotor.get("three");
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    boolean override = false;
    boolean aPressed = false;

    @Override
    public void loop() {
        // Controller values
        double r = gamepad1.right_stick_x / 2;
        double r2 = gamepad2.right_stick_x / 2;
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;
        double leftStickX1 = gamepad1.left_stick_x;
        double leftStickY1 = gamepad1.left_stick_y;
        double leftStickX2 = gamepad2.left_stick_x;
        double leftStickY2 = gamepad2.left_stick_y;


        if (gamepad2.a && !aPressed) {
            aPressed = true;
            override = !override;
        } else if (!gamepad2.a) {
            aPressed = false;
        }

        double x, y;

        if (!override) {
            x = leftStickX1;
            y = -leftStickY1;
        } else {
            x = leftStickX2;
            y = -leftStickY2;
            r = r2;
        }

        double scale = abs(r) + abs(y) + abs(x);

        //scales the inputs when needed
        if (scale > 1) {
            y /= scale;
            x /= scale;
            r /= scale;
        }

        double motor1Power = (-1.0 / 2.0) * x + (Math.sqrt(3) / 2) * y + r;
        double motor2Power = (-1.0 / 2.0) * x - (Math.sqrt(3) / 2) * y + r;
        double motor3Power = x + r;

        if (gamepad1.left_stick_x != 0 ||
                gamepad1.left_stick_y != 0 ||
                gamepad1.right_stick_x != 0 ||
                gamepad2.left_stick_x != 0 ||
                gamepad2.left_stick_y != 0 ||
                gamepad2.right_stick_x != 0) {
            motorOne.setPower(motor1Power / speed);
            motorTwo.setPower(motor2Power / speed);
            motorThree.setPower(motor3Power / speed);

        } else {
            motorOne.setPower(0);
            motorTwo.setPower(0);
            motorThree.setPower(0);

        }

        if (leftBumper && !pressed) {
            speed += 0.1;
            pressed = true;
        }
        if (rightBumper && !pressed) {
            speed -= 0.1;
            pressed = true;
        } else if (pressed && !rightBumper && !leftBumper) {
            pressed = false;
        }
        if (speed < 1) {
            speed = 1;
        }

        telemetry.addData("speed", 1 / speed);
        telemetry.addData("override", override);

    }
}