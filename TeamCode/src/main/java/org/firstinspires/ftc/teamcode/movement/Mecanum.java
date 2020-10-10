package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.maximum;
import org.firstinspires.ftc.teamcode.utility.point;


public class Mecanum {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    private static final double maxTicksPerSec = 1024;
    private static final double wheelToCenter = 13;

    //top l, top r, bottom l, bottom r
    public Mecanum(DcMotorEx... motors){

        frontLeft   = motors[0];
        frontRight  = motors[1];
        backLeft    = motors[2];
        backRight   = motors[3];

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    //turn speed be in robot angle
    public void drive(double xSpeed, double ySpeed, double turnSpeed){
        // radians = circumference / radius
        turnSpeed /= wheelToCenter;

        point move = new point(xSpeed, ySpeed);
        move.normalize();

        maximum max = new maximum(
                move.y + move.x - turnSpeed,  //FL
                move.y - move.x + turnSpeed,        //FR
                move.y - move.x - turnSpeed,        //BL
                move.y + move.x + turnSpeed);       //BR
        max.squishIntoRange(1.0);

        frontLeft.setVelocity(  (int)(maxTicksPerSec * max.nums[0]));
        frontRight.setVelocity( (int)(maxTicksPerSec * max.nums[1]));
        backLeft.setVelocity(   (int)(maxTicksPerSec * max.nums[2]));
        backRight.setVelocity(  (int)(maxTicksPerSec * max.nums[3]));
    }
}
