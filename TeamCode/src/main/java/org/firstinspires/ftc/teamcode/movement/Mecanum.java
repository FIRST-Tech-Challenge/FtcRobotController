package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.maximum;
import org.firstinspires.ftc.teamcode.utility.point;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class Mecanum {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    private static final double maxTicksPerSec = 1024;
    private static final double wheelToCenter = 21;

    //top l, top r, bottom l, bottom r
    public Mecanum(HardwareMap hardwareMap){  //todo index out of bounds error
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

//        frontLeft   = motors[0];
//        frontRight  = motors[1];
//        backLeft    = motors[2];
//        backRight   = motors[3];

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    //turn speed be in robot angle
    public void drive(double xSpeed, double ySpeed, double turnSpeed){
        // radians = circumference / radius
        turnSpeed /= wheelToCenter;

        point move = new point(xSpeed, ySpeed);
        move.normalize();

        maximum max = new maximum(
                move.y + move.x + turnSpeed,  //FL
                move.y - move.x - turnSpeed,        //FR
                move.y - move.x + turnSpeed,        //BL
                move.y + move.x - turnSpeed);       //BR
        max.squishIntoRange(1.0);



        frontLeft.setVelocity(  (int)(maxTicksPerSec * max.nums[0]));
        frontRight.setVelocity( (int)(maxTicksPerSec * max.nums[1]));
        backLeft.setVelocity(   (int)(maxTicksPerSec * max.nums[2]));
        backRight.setVelocity(  (int)(maxTicksPerSec * max.nums[3]));

        // Debugging
//        double frontLeftVelocity = frontLeft.getVelocity();
//        double frontRightVelocity = frontRight.getVelocity();
//        double backLeftVelocity = backLeft.getVelocity();
//        double backRightVelocity = backRight.getVelocity();
//        telemetry.addData("Status", "frontLeft: " + Double.toString(frontLeftVelocity));
//        telemetry.addData("Status", "frontRight: " + Double.toString(frontRightVelocity));
//        telemetry.addData("Status", "backLeft: " + Double.toString(backLeftVelocity));
//        telemetry.addData("Status", "backRight: " + Double.toString(backRightVelocity));

    }
}
