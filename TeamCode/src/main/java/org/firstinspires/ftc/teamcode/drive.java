package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class drive extends LinearOpMode {
    private DcMotor frontLeftMotor; // location 0
    private DcMotor frontRightMotor; // location 1
    private DcMotor backLeftMotor; // location 3
    private DcMotor backRightMotor; // location 2
    private Servo servoFlag;

    @Override
    public void runOpMode()  {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        servoFlag = hardwareMap.get(Servo.class, "servoFlag");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        float defaultPower = 2;

        boolean servoFirstPos = true;
        boolean servoMiddlePos = false;
        boolean servoLastPos = false;

        waitForStart();

        servoFlag.setPosition(0);

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) / defaultPower;
            double backRightPower = ((y + x - rx) / denominator) / defaultPower;
            double frontRightPower = ((y - x - rx) / denominator) / defaultPower;
            double backLeftPower = ((y - x + rx) / denominator) / defaultPower;

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                if (servoFirstPos){
                    servoFlag.setPosition(0.4);
                    servoFirstPos = false;
                    servoMiddlePos = true;

                } else if (servoMiddlePos == true){
                    servoFlag.setPosition(0.5);
                    servoMiddlePos = false;
                    servoLastPos = true;
                }else if (servoLastPos == true){
                    servoFlag.setPosition(0.6);
                    servoLastPos = false;
                    servoFirstPos = true;
                }
            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                servoFlag.setPosition(0);
                telemetry.addData("B", gamepad1B_pressed);
            }

            telemetry.addData("X Value", x);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.update();

        }

    }
}
