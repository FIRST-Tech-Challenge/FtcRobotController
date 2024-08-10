package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Drive extends LinearOpMode {

    public static final double SERVO_MOVE_SPEED = 0.025;
    public static final double SERVO_ONE_POS = 0.5;
    public static final double SERVO_TWO_POS = 0.5;
    public static final double POWER_SCALE = 3;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo servo1 = hardwareMap.servo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("servo2");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        double servoOnePos = SERVO_ONE_POS;
        double servoTwoPos = SERVO_TWO_POS;

        waitForStart();

        while (opModeIsActive()) {

            double powerForward = -gamepad1.left_stick_y / POWER_SCALE;
            double powerTurn = gamepad1.right_stick_x / POWER_SCALE;
            double mecanumMovement = gamepad1.left_stick_x / POWER_SCALE;
            double armMovement = gamepad1.right_stick_y;


            double fLeftPower = (powerForward + powerTurn + mecanumMovement);
            double fRightPower = (powerForward - powerTurn - mecanumMovement);
            double bLeftPower = (powerForward + powerTurn - mecanumMovement);
            double bRightPower = (powerForward - powerTurn + mecanumMovement);

            double maxPower = maxAbsPower(fLeftPower,fRightPower,bLeftPower,bRightPower);

            if (maxPower > 1) {

                fLeftPower /= maxPower;
                fRightPower /= maxPower;
                bLeftPower /= maxPower;
                bRightPower /= maxPower;
            }

            if (gamepad1.left_bumper == true) {
                servoOnePos += SERVO_MOVE_SPEED;
                servoTwoPos -= SERVO_MOVE_SPEED;
            }

            if (gamepad1.right_bumper == true) {
                servoOnePos -= SERVO_MOVE_SPEED;
                servoTwoPos += SERVO_MOVE_SPEED;
            }


            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);
            arm.setPower(armMovement);
            servo1.setPosition(servoOnePos);
            servo2.setPosition(servoTwoPos);

            //telemetry.addLine(String.valueOf(gamepad1.right_stick_y));
            //telemetry.addLine(String.valueOf(armMovement));
            //telemetry.update();

        }
    }

    private double maxAbsPower(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return Math.abs(max);

    }
}
