package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class RFTurret extends RFMotor {

    private RFMotor rotationMotor;

    double MAX_ROTATION_TICKS = 570;

    LinearOpMode op;

    /*motor, rotateToAngle, getAngle*/
    public RFTurret(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode opMode, boolean resetPos, DcMotor.ZeroPowerBehavior zeroBehavior) {
        super(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        rotationMotor = new RFMotor(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        op = opMode;

    }

    public void rotateToAngle (double targetAngle) {
        targetAngle /= (18.0/116);
        op.telemetry.addData("newAngle",targetAngle);
        int curPos = (int)getAngle();
        if(targetAngle>MAX_ROTATION_TICKS){
            targetAngle = MAX_ROTATION_TICKS - 5;
        }
        if(targetAngle<-MAX_ROTATION_TICKS){
            targetAngle = -MAX_ROTATION_TICKS + 5;
        }
        double distance = targetAngle-getCurrentPosition();
        while (Math.abs(distance) > 20) {

            distance = targetAngle-getCurrentPosition();
            op.telemetry.addData("current position:", getCurrentPosition());
            setVelocity(distance/abs(distance) * 4 * (abs(distance) + 100));
            op.telemetry.addData("distance", distance);
            op.telemetry.update();
        }
        setVelocity(0);
    }
    public void setVelocity(double target){rotationMotor.setVelocity(target);}

    public double getAngle() {
        return rotationMotor.getCurrentPosition();
    }
}