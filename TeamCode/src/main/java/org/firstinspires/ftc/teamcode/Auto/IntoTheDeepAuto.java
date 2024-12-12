package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.mainEnum;

@Autonomous(name = "Into the Deep Autonomous", group = "final")
public class IntoTheDeepAuto extends LinearOpMode implements AutoInterface {
    hardware hardware = new hardware();
    calculations calculations = new calculations();

    private int posFL;
    private int posFR;
    private int posBL;
    private int posBR;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes
        armInit();
        wheelInit();
        sensorInit();
        servoInit();

        // Resets encoder count
        resetMotorEncoders();

        // Direction
        setArmDirection();
        setWheelDirection();

        // Brakes
        armBrake();
        wheelBrake();

        // Telemetry
        telemetry();
        resetMotorEncoders();

        // Our base positions for all wheels are 0
        posFL = 0;
        posFR = 0;
        posBL = 0;
        posBR = 0;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Code Starting");
            test(calculations.forward10cm, calculations.forward10cm, calculations.forward10cm, calculations.forward10cm,
                    calculations.driveSpeed, calculations.driveSpeed, calculations.driveSpeed, calculations.driveSpeed);
        }
    }

    // Initialization functions
    @Override
    public void armInit() {
        hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
        hardware.lift = hardwareMap.get(DcMotor.class, "lift");
        hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
    }

    @Override
    public void wheelInit() {
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void sensorInit() {
        hardware.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        hardware.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");
        hardware.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        hardware.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
    }

    @Override
    public void servoInit() {
        hardware.wrist = hardwareMap.get(DcMotor.class, "wrist");
        hardware.door = hardwareMap.get(Servo.class, "door");
        hardware.topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
        hardware.bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
    }

    // Direction setup
    @Override
    public void setArmDirection() {
        hardware.lift.setDirection(DcMotor.Direction.REVERSE);
        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
        hardware.hopper.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void setWheelDirection() {
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    // Braking functions
    @Override
    public void armBrake() {
        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void wheelBrake() {
        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Motor position and speed
    @Override
    public void setWheelPosition(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR) {
        hardware.frontLeft.setTargetPosition(targetPosFL);
        hardware.frontRight.setTargetPosition(targetPosFR);
        hardware.backLeft.setTargetPosition(targetPosBL);
        hardware.backRight.setTargetPosition(targetPosBR);
    }

    @Override
    public void wheelMotorToPosition() {
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setWheelSpeed(double speedFL, double speedFR, double speedBL, double speedBR) {
        hardware.frontLeft.setPower(speedFL);
        hardware.frontRight.setPower(speedFR);
        hardware.backLeft.setPower(speedBL);
        hardware.backRight.setPower(speedBR);
    }

    @Override
    public void setArmPosition(int targetPosMantis, int targetPosLift, int targetPosHopper) {
        hardware.mantis.setTargetPosition(targetPosMantis);
        hardware.lift.setTargetPosition(targetPosLift);
        hardware.lift.setTargetPosition(targetPosHopper);
    }

    @Override
    public void armMotorToPosition() {
        hardware.mantis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.hopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setArmSpeed(double speedMantis, double speedLift, double speedHopper) {
        // Implementation for setting arm speeds
    }

    @Override
    public void setClawSpeed(double speedWrist, double speedTopGrabber, double speedBottomGrabber) {
        // Implementation for setting claw speeds
    }

    @Override
    public void doorPos(double pos) {
        hardware.door.setPosition(pos);
    }

    // Miscellaneous
    @Override
    public void telemetry() {
        telemetry.addLine("Code is running");
        telemetry.update();
    }

    @Override
    public void whileMotorsBusy() {
        telemetry.addLine("Code is running");
    }

    @Override
    public void resetMotorEncoders() {
        hardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Base function
    @Override
    public void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR) {
        setWheelPosition(targetPosFL, targetPosFR, targetPosBL, targetPosBR);
        telemetry.addLine("Position set");
        telemetry.update();
        sleep(250);

        wheelMotorToPosition();
        telemetry.addLine("Running to position");
        telemetry.update();
        sleep(250);

        setWheelSpeed(speedFL, speedFR, speedBL, speedBR);
        telemetry.addLine("Setting power");
        telemetry.update();
        sleep(250);
        while (opModeIsActive() &&
                (hardware.frontLeft.getCurrentPosition() < targetPosFL ||
                        hardware.frontRight.getCurrentPosition() < targetPosFR ||
                        hardware.backLeft.getCurrentPosition() < targetPosBL ||
                        hardware.backRight.getCurrentPosition() < targetPosBR)) {
            telemetry.addLine("It's running");
            telemetry.addData("Back Right Position: ", hardware.backRight.getCurrentPosition());
            telemetry.addData("Back Left Position: ", hardware.backLeft.getCurrentPosition());
            telemetry.addData("Front Right Position: ", hardware.frontRight.getCurrentPosition());
            telemetry.addData("Front Left Position: ", hardware.frontLeft.getCurrentPosition());
            telemetry.update();
        }
        setWheelSpeed(0, 0, 0, 0);
        resetMotorEncoders();
    }

    // Test function
        private void test(int FLpos, int FRpos, int BLpos, int BRpos, double FLSpeed, double FRSpeed, double BLSpeed, double BRSpeed) {
            // Update cumulative target positions
            posFL += FLpos;
            posFR += FRpos;
            posBL += BLpos;
            posBR += BRpos;

            // Set the target positions for all motors
            hardware.frontLeft.setTargetPosition(posFL);
            hardware.frontRight.setTargetPosition(posFR);
        hardware.backLeft.setTargetPosition(posBL);
        hardware.backRight.setTargetPosition(posBR);

        // Set all motors to RUN_TO_POSITION mode
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply motor speeds
        hardware.frontLeft.setPower(FLSpeed);
        hardware.frontRight.setPower(FRSpeed);
        hardware.backLeft.setPower(BLSpeed);
        hardware.backRight.setPower(BRSpeed);

        // Define a margin of error for the encoder positions
        int threshold = 10;  // Allowable error in encoder ticks

        // Loop until all motors are within the threshold of their target positions
        while (opModeIsActive() &&
                (Math.abs(hardware.frontLeft.getCurrentPosition() - posFL) > threshold ||
                        Math.abs(hardware.frontRight.getCurrentPosition() - posFR) > threshold ||
                        Math.abs(hardware.backLeft.getCurrentPosition() - posBL) > threshold ||
                        Math.abs(hardware.backRight.getCurrentPosition() - posBR) > threshold)) {
            // Add telemetry for debugging
            telemetry.addData("Front Left Position", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", hardware.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position", hardware.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", hardware.backRight.getCurrentPosition());
            telemetry.update();

            idle();  // Prevent the loop from hogging CPU resources
        }

        // Stop all motors once target positions are reached
        hardware.frontLeft.setPower(0);
        hardware.frontRight.setPower(0);
        hardware.backLeft.setPower(0);
        hardware.backRight.setPower(0);

        // Reset motor encoders for the next movement
        resetMotorEncoders();
    }

}
