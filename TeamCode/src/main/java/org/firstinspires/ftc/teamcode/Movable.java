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

    //    static protected Servo ServoR;
//    static protected Servo ServoL;

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        ScissorLiftR = hardwareMap.get(DcMotor.class, "ScissorLiftR"); // change config if needed
        ScissorLiftL = hardwareMap.get(DcMotor.class, "ScissorLiftL");
//        ServoR = hardwareMap.get(Servo.class, "ServoR");
//        ServoL = hardwareMap.get(Servo.class, "ServoL");

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
        ScissorLiftL.setPower(0);
        ScissorLiftR.setPower(0);
    }
    public void disablePower() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    public void powerScissorLift(int miliseconds, String direction) {
        switch (direction) {
            case "up":
                ScissorLiftL.setPower(1);
                ScissorLiftR.setPower(1);
                sleep(miliseconds);
                disableScissorPower();
                break;
            case "down":
                ScissorLiftL.setPower(-1);
                ScissorLiftR.setPower(-1);
                sleep(miliseconds);
                disableScissorPower();
                break;
        }
    }
//
//    public void claw(String toggle) {
//        switch (toggle) {
//            case "open":
//                ServoR.setPosition(1);
//                ServoL.setPosition(1);
//                break;
//            case "close":
//                ServoR.setPosition(0);
//                ServoL.setPosition(0);
//                break;
//        }
//    }

    abstract void updatePhoneConsole();
}
