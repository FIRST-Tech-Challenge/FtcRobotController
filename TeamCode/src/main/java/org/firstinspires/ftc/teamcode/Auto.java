package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;


public class Auto extends LinearOpMode {
    private final int SERVO_MAX_POSITION = 90;
    private DcMotor frontLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor backRightWheel = null;

    //MODIFY LATER ONCE PHONES ARE CONFIGED
    private String flWheelName = "";
    private String frWheelName = "";
    private String blWheelName = "";
    private String brWheelName = "";
    private DcMotor[] motorList = {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel};
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftWheel = hardwareMap.get(DcMotor.class, flWheelName);
        frontRightWheel = hardwareMap.get(DcMotor.class, frWheelName);
        backLeftWheel = hardwareMap.get(DcMotor.class, blWheelName);
        backRightWheel = hardwareMap.get(DcMotor.class, brWheelName);
        //tests
        configWheels();
        testRun();

    }
    private void modAllWheels(String action, Float... power) {
        /*
        if (action.equals("direction_forward")) {
            for (wheel : motorList) {
                wheel.setDirection(DcMotor.Direction.FORWARD)
            }
        }
        if (action.equals("direction_reverse")) {
            for (wheel : motorList) {
                wheel.setDirection(DcMotor.Direction.REVERSE)
            }
        }*/
        switch (action) {
            case "direction_forward":
                for (wheel : motorList) {
                    wheel.setDirection(DcMotor.Direction.FORWARD);
                }
                break;
            case "direction_reverse":
                for (wheel : motorList) {
                    wheel.setDirection(DcMotor.Direction.FORWARD);
                }
                break;
            case "power":
                for (wheel : motorList) {
                    wheel.setPower(power[0]);
                }
                break;
            default:
                break;
        }
            
                
    }
    private void configWheels() {
        modAllWheels("direction_forward");
    }
    private void testRun() {
        modAllWheels("power", 1);

    }
}
