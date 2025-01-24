package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    static protected Servo outtakeR;
    static protected Servo outtakeL;
    static protected Servo intake;

    static protected boolean inverse = false;
    static protected boolean stop = false;

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
        outtakeR = hardwareMap.get(Servo.class, "outtakeR"); // change if needed
        outtakeL = hardwareMap.get(Servo.class, "outtakeL");
        intake = hardwareMap.get(Servo.class, "intake"); // change if needed
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
    // 0 right
    // 5 left
    public void powerScissorLift(int milliseconds, String direction) {
        switch (direction) {
            case "up":
                FRScissorLift.setPower(.9);
                FLScissorLift.setPower(1);
                BRScissorLift.setPower(-.9);
                BLScissorLift.setPower(-1);
                sleep(milliseconds);
                disableScissorPower();
                break;
            case "down":
                FRScissorLift.setPower(-.9);
                FLScissorLift.setPower(-1);
                BRScissorLift.setPower(.9);
                BLScissorLift.setPower(1);
                sleep(milliseconds);
                disableScissorPower();
                break;
        }
    }

    // change values if needed
    public void outtakeGrab(String toggle) {
        switch (toggle) {
            case "constriction":
                outtakeR.setPosition(1);
                outtakeL.setPosition(1);
                break;
            case "liberation":
                outtakeR.setPosition(0);
                outtakeL.setPosition(0);
                break;
        }
        try {
            wait(500);
        } catch (Exception e) {

        }
    }

    // change values if needed
    public void inttakeGrab(String toggle) {
        switch (toggle) {
            case "constriction":
                intake.setPosition(1);
                break;
            case "liberation":
                intake.setPosition(0);
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

    public void inverseControls() {
        try {
            inverse = !inverse;
            wait(50);
        } catch (Exception e) {

        }
    }

    // change values if needed
    public void turnRotatingServos(String toggle) {
        rotatingServoL.setDirection(Servo.Direction.REVERSE);
        switch (toggle) {
            case "rollUp":
                SlideR.setPosition(1);
                SlideL.setPosition(1);
                break;
            case "rollDown":
                SlideR.setPosition(0);
                SlideL.setPosition(0);
                break;
        }
        try {
            wait(500);
        } catch (Exception e) {

        }
    }

    public void turn180() {
        turnRobot(2940,"right");
    }

    public void turn90(String direction) {
        turnRobot(2940/2,direction);
    }

    abstract void updatePhoneConsole();
}