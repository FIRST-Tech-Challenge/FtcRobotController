package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.maximum;
import org.firstinspires.ftc.teamcode.utility.point;


public class Mecanum {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    //for debugging
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;

    private static final double maxTicksPerSec = 2000;
    private static final double wheelToCenter = 21;

    //top l, top r, bottom l, bottom r
    public Mecanum(HardwareMap hardwareMap){  //todo bring back the list of motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

//        frontLeft   = motors[0];
//        frontRight  = motors[1];
//        backLeft    = motors[2];
//        backRight   = motors[3];

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
    }
//    // for more debugging
//    double getFrontLeftSpeed(){ return frontLeftSpeed; }
//    double getBackLeftSpeed(){ return backLeftSpeed; }
//    double getFrontRightSpeed(){ return frontRightSpeed; }
//    double getBackRightSpeed(){ return backRightSpeed; }

    //turn speed be in robot angle

//    public void driveTeleopPower(double xSpeed, double ySpeed, double turnSpeed) {
//
//        double frontLeftSpeed = ySpeed + xSpeed + turnSpeed;
//        double frontRightSpeed = ySpeed - xSpeed - turnSpeed;
//        double backLeftSpeed = ySpeed - xSpeed + turnSpeed;
//        double backRightSpeed = ySpeed + xSpeed - turnSpeed;
//
//        frontLeft.setPower(frontLeftSpeed);
//        frontRight.setVelocity(frontRightSpeed);
//        backLeft.setVelocity(backLeftSpeed);
//        backRight.setVelocity(backRightSpeed);
//    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed){
        // radians = circumference / radius
        //turnSpeed /= wheelToCenter;

//        point move = new point(xSpeed, ySpeed);
//
//        maximum max = new maximum(
//                move.y + move.x + turnSpeed,  //FL
//                move.y - move.x - turnSpeed,        //FR
//                move.y - move.x + turnSpeed,        //BL
//                move.y + move.x - turnSpeed);       //BR
        //max.squishIntoRange(1.0);



        frontLeft.setVelocity(  (int)(maxTicksPerSec * (ySpeed - xSpeed + turnSpeed)));
        frontRight.setVelocity( (int)(maxTicksPerSec * (ySpeed - xSpeed - turnSpeed)));
        backLeft.setVelocity(   (int)(maxTicksPerSec * (ySpeed + xSpeed + turnSpeed)));
        backRight.setVelocity(  (int)(maxTicksPerSec * (ySpeed + xSpeed - turnSpeed)));

//        //for debugging
//        frontLeftSpeed = frontLeft.getVelocity();
//        frontRightSpeed = frontRight.getVelocity();
//        backLeftSpeed = backLeft.getVelocity();
//        backRightSpeed = backRight.getVelocity();
    }


}
