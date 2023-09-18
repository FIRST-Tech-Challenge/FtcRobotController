package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Hello World",group="Robot")
public class Hello_World extends LinearOpMode {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    public DcMotor liftMotor = null;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double GEAR_RATIO = 3/5;    // The gear ratio of the gears attached to the motor.

    static final double INCHES_PER_DEGREE = (11.0/90);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (GEAR_RATIO);

    //static final double INCHES_PER_COUNT = (WHEEL_DIAMETER_INCH * 3.1415) / (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
    // for testing




    public Servo Claw = null;

    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World");
        telemetry.update();
        leftMotor = hardwareMap.get(DcMotor.class, "MotorA");
        rightMotor = hardwareMap.get(DcMotor.class, "MotorB");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Claw = hardwareMap.get(Servo.class, "Servo1");

        liftMotor = hardwareMap.get(DcMotor.class, "MotorC");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double leftSpeed;
        double rightSpeed;
        double clawPosition = 0;
        double liftSpeed;
        boolean macro = false;
        int targetPosition = 0;


        waitForStart();



        while (opModeIsActive()) {
            if (gamepad1.b) {
                telemetry.addData("gamepad1.b", gamepad1.b);
            }

            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            leftMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);


            if (gamepad1.right_trigger >= 0.4) {
                liftMotor.setPower(gamepad1.right_trigger);
                telemetry.addData("liftMotor.getCurrentPosition", liftMotor.getCurrentPosition());
            } else if (gamepad1.left_trigger >= 0.4) {
                liftMotor.setPower(gamepad1.left_trigger * -1);
                telemetry.addData("liftMotor.getCurrentPosition", liftMotor.getCurrentPosition());
            } else if (!macro) {
                liftMotor.setPower(0);
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                telemetry.addLine("Claw Opened");
                clawPosition = 0.3;
                Claw.setPosition(clawPosition);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                //don't need    telemetry.addData("gamepad1.right_bumper",gamepad1.right_bumper);
                telemetry.addLine("claw closed");
                clawPosition = 0.7;
                Claw.setPosition(clawPosition);
            }

            if (gamepad1.a) {
                targetPosition = (int) (13.5*COUNTS_PER_INCH);
                macro = true;
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setPower(1);

                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))>=10) {
                telemetry.addLine("Macro Running!");
                telemetry.addData("Ticks remaining", liftMotor.getCurrentPosition()-targetPosition);
            }
            else if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))<=10) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                macro = false;
            }
            if (gamepad1.b) {
                targetPosition = ((int)(23.5*COUNTS_PER_INCH));
                macro = true;
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setPower(1);

                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))>=10) {
                telemetry.addLine("Macro Running!");
                telemetry.addData("Ticks remaining", liftMotor.getCurrentPosition()-targetPosition);
            }
            else if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))<=10) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                macro = false;
            }
            if (gamepad1.x) {
                targetPosition = (0);
                macro = true;
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setPower(1);

                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))>=10) {
                telemetry.addLine("Macro Running!");
                telemetry.addData("Ticks remaining", liftMotor.getCurrentPosition()-targetPosition);
            }
            else if (macro && (Math.abs(liftMotor.getCurrentPosition()-targetPosition))<=10) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                macro = false;
            }


            telemetry.addData("Ticks",liftMotor.getCurrentPosition());
            telemetry.addData("Macro Enabled", macro);

            telemetry.update();
            sleep(50);

        }
    }
}

