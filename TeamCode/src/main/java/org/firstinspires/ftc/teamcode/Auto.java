package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.
import com.qualcomm.robotcore.hardware.DcMotor;


public class Auto extends LinearOpMode{
    private int servoportmax = 90;
    private DcMotor frontLeftWheel;
    private DcMotor frontRightWheel;
    private DcMotor backLeftWheel;
    private DcMotor backRightWheel;
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
        testWheels();

    }
    private void testWheels() {
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

    }

}
