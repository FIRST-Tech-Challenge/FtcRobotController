package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonHandler;

@TeleOp
public class newBotTeleOp extends LinearOpMode {
    private DcMotor middleMotor; // location 0 - eh
    private DcMotor frontLeftMotor; // location 0 - ch
    private DcMotor frontRightMotor; // location 1 - ch
    private DcMotor backRightMotor; // location 2 - ch
    private DcMotor backLeftMotor; // location 3 - ch
    private DcMotor jointMotor; // location 1 - eh
    private DcMotor slideMotor; // location 2 - eh
    private Servo claw;
    private ElapsedTime newTimer = new ElapsedTime();
    private DigitalChannel green0; //0
    private DigitalChannel red0; //1
    @Override
    public void runOpMode()  {

        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        claw = hardwareMap.get(Servo.class,"claw");

        green0 = hardwareMap.get(DigitalChannel.class, "green0");
        red0 = hardwareMap.get(DigitalChannel.class, "red0");

        green0.setMode(DigitalChannel.Mode.INPUT);
        red0.setMode(DigitalChannel.Mode.INPUT);


        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Reverse the other motors and sex X to not negative

        float defaultPower = 2;
        double changeSpeed = 1;
        double middleDefaultPower = 1.25;

        boolean servoFirstPos = true;
        boolean servoMiddlePos = false;
        boolean servoLastPos = false;

        boolean changeSpeedPos = false;

        waitForStart();

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newTimer.reset();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double middleMotorPower = ((rx)) / middleDefaultPower;
            double frontLeftMotorPower = ((y + x) / denominator) / defaultPower;
            double backLeftMotorPower = ((y + x) / denominator) / defaultPower;
            double frontRightMotorPower = ((y - x) / denominator) / defaultPower;
            double backRightMotorPower = ((y - x) / denominator) / defaultPower;

            middleMotor.setPower(middleMotorPower / changeSpeed);
            frontLeftMotor.setPower(frontLeftMotorPower / changeSpeed);
            frontRightMotor.setPower(frontRightMotorPower / changeSpeed);
            backRightMotor.setPower(backRightMotorPower / changeSpeed);
            backLeftMotor.setPower(backLeftMotorPower / changeSpeed);

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                if (changeSpeedPos){
                    changeSpeedPos = false;
                    changeSpeed = 1;

                } else {
                    changeSpeed = 2;
                    changeSpeedPos = true;
                }


            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                telemetry.addData("B", gamepad1B_pressed);
            }
            if (gamepad2.dpad_down) {
                jointMotor.setTargetPosition(100);
                jointMotor.setPower(0.5);

            }
            if (gamepad2.dpad_up) {
                jointMotor.setTargetPosition(0);
                jointMotor.setPower(0.5);

            }








            if (newTimer.seconds() >= 10){
                telemetry.addLine("10");

                green0.setMode(DigitalChannel.Mode.INPUT);
                red0.setMode(DigitalChannel.Mode.OUTPUT);

            }else if (newTimer.seconds() >= 5){
                green0.setMode(DigitalChannel.Mode.OUTPUT);
                red0.setMode(DigitalChannel.Mode.INPUT);

                telemetry.addLine("5");

            }else{

                telemetry.addLine("0");



            }

            telemetry.addData("X Value", x);
            telemetry.addData("time", newTimer.seconds());
            telemetry.addData("jointMotorpos", jointMotor.getCurrentPosition());
            telemetry.update();

        }

    }
}
