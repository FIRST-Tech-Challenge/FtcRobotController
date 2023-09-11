package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * THIS IS JAMES'S CODE
 */

//This code initializes the motors and servos. If you make your TeleOp class inherit from this class, all of your stuff will be initiated.
//The code pretty much lets us be a bit lazy.

//TODO: Change the motor/servo variables to initialize what your robot has. The wheel initializations should be the same for both teams for simplicity.

@Disabled
@TeleOp(name="RobotCore", group="Core")

public class RobotCore extends OpMode{

    //variables for motors and servos
    DcMotor leftFront=null; //Currently Slide
    DcMotor rightFront=null;
    DcMotor leftRear=null;
    DcMotor rightRear=null;

    //IMU variables
    BNO055IMU inertiaMeasure;
    //heading
    double heading;
    //Calibrates IMU
    String imuCal="RevIMUCalibration.json";

    //initializes the base robot

    @Override
    public void init(){
        //initialize drive motors
        leftFront=hardwareMap.get(DcMotor.class, "leftFront");
        rightFront=hardwareMap.get(DcMotor.class, "rightFront");
        leftRear=hardwareMap.get(DcMotor.class, "leftRear");
        rightRear=hardwareMap.get(DcMotor.class, "rightRear");

        //TODO: Reverse or set specific motor behaviors here after you initialize them.

        leftRear.setDirection((DcMotor.Direction.REVERSE));
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //These methods are to be overridden in the classes
    @Override
    public void start(){

    }

    @Override
    public void loop(){

    }

    @Override
    public void stop(){

    }
}