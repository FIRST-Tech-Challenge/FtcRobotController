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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // front right wheel is the only one WRONG
    public void powerWheels(int miliseconds, String direction) {
        switch (direction) {
            case "forward":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(-0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "backward":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.3);
                BLW.setPower(-0.3);
                FRW.setPower(0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "left":
                FLW.setPower(-0.3);
                BLW.setPower(0.3);
                FRW.setPower(-0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
                disablePower();
                break;
        }
    }

    public void turnRobot(int miliseconds, String direction) {
        switch (direction) {
            case "left":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(-0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
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
    public void powerScissorLift(int miliseconds, String direction) {
        switch (direction) {
            case "up":
                FRScissorLift.setPower(-1);
                FLScissorLift.setPower(-.975);
                BRScissorLift.setPower(1);
                BLScissorLift.setPower(.975);
                sleep(miliseconds);
                disableScissorPower();
                break;
            case "down":
                FRScissorLift.setPower(1);
                FLScissorLift.setPower(.975);
                BRScissorLift.setPower(-1);
                BLScissorLift.setPower(-.975);

                sleep(miliseconds);
                disableScissorPower();
                break;
        }
    }

    public void moveClaws(String toggle) {
        SlideL.setDirection(Servo.Direction.REVERSE);
        switch (toggle) {
            case "grab":
                SlideR.setPosition(.5);
                SlideL.setPosition(.5);
                break;
            case "release":
                SlideR.setPosition(0);
                SlideL.setPosition(0);
                break;
        }
    }


    abstract void updatePhoneConsole();
}