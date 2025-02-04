package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

abstract class Movable extends LinearOpMode {
    static protected DcMotor FLW;
    static protected DcMotor BLW;
    static protected DcMotor FRW;
    static protected DcMotor BRW;
    static protected DcMotor FRScissorLift;
    static protected DcMotor BRScissorLift;
    static protected DcMotor FLScissorLift;
    static protected DcMotor BLScissorLift;
    static protected Servo SlideR;
    static protected Servo SlideL;
    static protected Servo rotatingServoR;
    static protected Servo rotatingServoL;
    static protected Servo outtake;
    static protected Servo intake;

    static protected Delay outtakeDelay = new Delay();
    static protected Delay inttakeDelay = new Delay();
    static protected Delay rollingDelay = new Delay();
    static protected Delay slideDelay = new Delay();

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        FRScissorLift = hardwareMap.get(DcMotor.class, "FRScissorLift");
        FLScissorLift = hardwareMap.get(DcMotor.class, "FLScissorLift");
        BRScissorLift = hardwareMap.get(DcMotor.class, "BRScissorLift");
        BLScissorLift = hardwareMap.get(DcMotor.class, "BLScissorLift");
        SlideR = hardwareMap.get(Servo.class, "SlideR");
        SlideL = hardwareMap.get(Servo.class, "SlideL");
        rotatingServoR = hardwareMap.get(Servo.class, "rotatingServoR"); // change if needed
        rotatingServoL = hardwareMap.get(Servo.class, "rotatingServoL");
        outtake = hardwareMap.get(Servo.class, "outtakeClaw"); // change if needed
        intake = hardwareMap.get(Servo.class, "intakeClaw"); // change if needed
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // front right wheel is the only one WRONG
    public void powerWheels(int milliseconds, String direction) {
        switch (direction) {
            case "forward":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(-0.3);
                BRW.setPower(-0.3);
                sleep(milliseconds);
                disablePower();
                break;
            case "backward":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(0.3);
                BRW.setPower(0.3);
                sleep(milliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.7);
                BLW.setPower(-0.7);
                FRW.setPower(0.7);
                BRW.setPower(-0.7);
                sleep(milliseconds);
                disablePower();
                break;
            case "left":
                FLW.setPower(-0.7);
                BLW.setPower(0.7);
                FRW.setPower(-0.7);
                BRW.setPower(0.7);
                sleep(milliseconds);
                disablePower();
                break;
        }
    }

    public void turnRobot(int milliseconds, String direction) {
        switch (direction) {
            case "left":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(-0.3);
                BRW.setPower(-0.3);
                sleep(milliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(0.3);
                BRW.setPower(0.3);
                sleep(milliseconds);
                disablePower();
                break;
        }
    }

    public void disableScissorPower() {
        FRScissorLift.setPower(0);
        BRScissorLift.setPower(0);
        FLScissorLift.setPower(0);
        BLScissorLift.setPower(0);
    }
    public void disablePower() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    public void powerScissorLift(int miliseconds, String direction) {
        final double LPOWER = .96;
        final double RPOWER = 1;

        switch (direction) {
            case "erect":
                FRScissorLift.setPower(-RPOWER);
                FLScissorLift.setPower(-LPOWER);
                BRScissorLift.setPower(RPOWER);
                BLScissorLift.setPower(LPOWER);
                sleep(miliseconds);
                disableScissorPower();
                break;
            case "descend":
                FRScissorLift.setPower(RPOWER);
                FLScissorLift.setPower(LPOWER);
                BRScissorLift.setPower(-RPOWER);
                BLScissorLift.setPower(-LPOWER);
                sleep(miliseconds);
                disableScissorPower();
                break;
        }
    }

    // change values if needed
    public void outtakeGrab(String toggle) {
        switch (toggle) {
            case "liberation":
                outtake.setPosition(.3);
                break;
            case "constriction":
                outtake.setPosition(0);
                break;
        }
    }

    // change values if needed
    public void intakeGrab(String toggle) {
        switch (toggle) {
            case "constriction":
                intake.setPosition(.7);
                break;
            case "liberation":
                intake.setPosition(.4);
                break;
        }
        try {
            wait(500);
        } catch (Exception e) {
            
        }
    }

    public void moveSlides(String toggle) {
        SlideL.setDirection(Servo.Direction.REVERSE);
        switch (toggle) {
            case "thrust":
                SlideR.setPosition(1);
                SlideL.setPosition(1);
                break;
            case "retract":
                SlideR.setPosition(0);
                SlideL.setPosition(0);
                break;
        }
        try {
            wait(500);
        } catch (Exception e) {

        }
    }

    // change values if needed
    public void turnRotatingServos(String toggle) {
        rotatingServoL.setDirection(Servo.Direction.REVERSE);
        switch (toggle) {
            case "rollUp":
                rotatingServoR.setPosition(.62);
                rotatingServoL.setPosition(.62);
                break;
            case "rollDown":
                rotatingServoR.setPosition(.08);
                rotatingServoL.setPosition(.08);
                break;
        }
        try {
            wait(500);
        } catch (Exception e) {

        }
    }

    public void turn180() {
        turnRobot(2990,"right");
    }

    public void turn90(String direction) {
        turnRobot(2990/2, direction);
    }

    abstract void updatePhoneConsole();
}