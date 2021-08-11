package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

@Autonomous(name = "odomCalibration", group = "odomTest")
public class OdomCalibration extends UpliftAuto {

    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        while(robot.imuAngle <= 90) {
            lf.setPower(0.5);
            lb.setPower(0.5);
            rf.setPower(-0.5);
            rb.setPower(-0.5);
        }
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        robot.safeSleep(1000);
        Log.i("angle", robot.worldAngle + "");
        Log.i("x", robot.worldX + "");
        Log.i("y", robot.worldY + "");
        Log.i("imuAngle", robot.imuAngle + "");

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
