package com.team16488.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class makes it really easy for us to initialize our robots hardware
 * here we declare all the hardware variables that we will be using in the robot and initilize them in
 * our robots init method, see robot.java for more
 * when writing each individual component we initialize this method IN THE CONSTRUCTOR OF THE COMPONENT that allows us
 * to store all of our hardware variables place and just call them from each class
 *
 * Note: when making a component we only have to call from here to get the hardware objects afterwards we can use local objecs or these objects
 */
public class RobotMap {
    // hardware map object(local, hence why its private)
    HardwareMap map;

    // Robot objects(project wide, hence why its public)
    // drivetrain motors
    public DcMotor FrontRightMotor  = null;
    public DcMotor FrontLeftMotor   = null;
    public DcMotor RearRightMotor   = null;
    public DcMotor RearLeftMotor    = null;

    public RobotMap(){
        // constructor
    }

    public void mapHardware(HardwareMap hardwareMap){
        // here we map all of our hardware components
        map = hardwareMap;

        //---------------------------------------------------------------
        // Drive Train
        //---------------------------------------------------------------
        //Drive train
        FrontLeftMotor  = map.get(DcMotor.class, "FL");
        FrontRightMotor = map.get(DcMotor.class, "FR");
        RearRightMotor  = map.get(DcMotor.class, "BR");
        RearLeftMotor   = map.get(DcMotor.class, "BL");

        //set the left side direction to reverse
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //set all powers to zero
        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);
    }

}
