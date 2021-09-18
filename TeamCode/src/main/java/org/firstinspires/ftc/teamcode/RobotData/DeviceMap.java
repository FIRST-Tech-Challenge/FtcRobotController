package org.firstinspires.ftc.teamcode.RobotData;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//Testing
public class DeviceMap {
    //Listing hardware devices
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public Servo frontClaw = null;
    public RevBlinkinLedDriver light = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public DeviceMap() {
    }
    {
                        // Naming hardware Devices
                        //Test
                        //Testing Again
                        //test
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        frontClaw = hwMap.get(Servo.class, "frontClaw");
        light = hwMap.get(RevBlinkinLedDriver.class, "Led");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        //Setting Power to motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
            //Setting to run without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    }
}