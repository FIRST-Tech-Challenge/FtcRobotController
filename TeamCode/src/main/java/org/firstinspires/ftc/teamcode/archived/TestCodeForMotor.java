package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//CRServo
@Disabled
public class TestCodeForMotor extends LinearOpMode {
    private CRServo intakeServo;
    private DcMotor armMotor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        //frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        //frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        telemetry.addData("Hello",", Team RoboActive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Status", "Game Started...");

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armMotor.setPower(0);
            //frontLeft.setPower(0.1);
            //frontRight.setPower(0.1);
            backLeft.setPower(0);
            backRight.setPower(0);

            //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            } if (gamepad2.dpad_up) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                //frontLeft.setPower(0.1);
                //frontRight.setPower(0.1);
                backLeft.setTargetPosition(1);
                backRight.setTargetPosition(1);
                //frontLeft.setTargetPosition(1);
                //frontRight.setTargetPosition(1);
                telemetry.addData("Running", "forwards");
                telemetry.update();
                //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.dpad_down) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                //frontLeft.setPower(0.1);
                //frontRight.setPower(0.1);
                backLeft.setTargetPosition(-1);
                backRight.setTargetPosition(-1);
                //frontLeft.setTargetPosition(-1);
                //frontRight.setTargetPosition(-1);
                telemetry.addData("Running", "backwards");
                telemetry.update();
                //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.dpad_left) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                backLeft.setTargetPosition(-1);
                backRight.setTargetPosition(-1);
                telemetry.addData("Running", "leftTurn");
                telemetry.update();
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.dpad_right) {
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                backLeft.setTargetPosition(-1);
                backRight.setTargetPosition(1);
                telemetry.addData("Running", "rightTurn");
                telemetry.update();
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            waitForStart();

        }
    }


