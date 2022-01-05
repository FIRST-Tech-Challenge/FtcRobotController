package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AcRobot {
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    public AcRobot(){
    }

    //run this before anything else
    public void initHardware(HardwareMap hardwareMap){

        //initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        //set two of the motors to be reversed
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    // x and y are a vector direction
    public void DRIVE_THE_FRIKING_ROBOT(double x, double y){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = -x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }


}
