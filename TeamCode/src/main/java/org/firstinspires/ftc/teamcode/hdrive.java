package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class hdrive extends LinearOpMode {
    private DcMotor middleMotor; // location 0
    private DcMotor leftMotor; // location 1
    private DcMotor rightMotor; // location 2
    private Servo servoClaw;
    private ElapsedTime newTimer = new ElapsedTime();
    private DigitalChannel green; //0
    private DigitalChannel red; //1
    @Override
    public void runOpMode()  {

        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        green = hardwareMap.get(DigitalChannel.class, "green");
        red = hardwareMap.get(DigitalChannel.class, "red");

        green.setMode(DigitalChannel.Mode.INPUT);
        red.setMode(DigitalChannel.Mode.INPUT);


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reverse the other motors and sex X to not negative

        float defaultPower = 2;

        boolean servoFirstPos = true;
        boolean servoMiddlePos = false;
        boolean servoLastPos = false;

        waitForStart();


        newTimer.reset();

        servoClaw.setPosition(0.35);

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double middleMotorPower = ((rx)) / defaultPower;
            double leftMotorPower = ((y + x) / denominator) / defaultPower;
            double rightMotorPower = ((y - x) / denominator) / defaultPower;

            middleMotor.setPower(middleMotorPower);
            leftMotor.setPower(leftMotorPower);
            rightMotor.setPower(rightMotorPower);

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            green.setState(true);

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                if (servoFirstPos){
                    servoClaw.setPosition(0.35);
                    green.setState(true);
                    servoFirstPos = false;
                    servoMiddlePos = true;

                } else if (servoMiddlePos){
                    servoClaw.setPosition(0.5);
                    red.setState(true);
                    servoMiddlePos = false;
                    servoLastPos = true;
                }else if (servoLastPos){
                    servoClaw.setPosition(0.65);
                    green.setState(false);
                    red.setState(false);
                    servoLastPos = false;
                    servoFirstPos = true;
                }
            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                servoClaw.setPosition(0);
                telemetry.addData("B", gamepad1B_pressed);
            }
            if (newTimer.seconds() >= 10){
                telemetry.addData("10","test");

                green.setMode(DigitalChannel.Mode.INPUT);
                red.setMode(DigitalChannel.Mode.OUTPUT);

            }else if (newTimer.seconds() >= 5){
                green.setMode(DigitalChannel.Mode.OUTPUT);
                red.setMode(DigitalChannel.Mode.INPUT);

                telemetry.addData("5","test");

            }else{

                telemetry.addData("0","test");



            }

            telemetry.addData("X Value", x);
            telemetry.addData("middleMotorPower", middleMotorPower);
            telemetry.addData("leftMotorPower", leftMotorPower);
            telemetry.addData("backLeftPower", rightMotorPower);
            telemetry.addData("time", newTimer.seconds());
            telemetry.update();

        }

    }
}
