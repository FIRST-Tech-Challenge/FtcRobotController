package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static org.firstinspires.ftc.teamcode.Robot.checker;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretRotationStates.TURRET_ROTATED;
import static org.firstinspires.ftc.teamcode.Components.Turret.rotatePosition;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFTurret extends RFMotor {



    private RFMotor rotationMotor;

    double MAX_ROTATION_TICKS = 570;

    LinearOpMode op;

    /*motor, rotateToAngle, getAngle*/
    public RFTurret(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode opMode, boolean resetPos, DcMotor.ZeroPowerBehavior zeroBehavior, Logger log) {
        super(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior, log);

        rotationMotor = new RFMotor(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior, log);

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

    public void TurretRotate (double targetAngle) {
        op.telemetry.addData("newAngle",targetAngle);
        int curPos = (int)rotatePosition;
        if(targetAngle>MAX_ROTATION_TICKS){
            targetAngle = MAX_ROTATION_TICKS;
        }
        if(targetAngle<-MAX_ROTATION_TICKS){
            targetAngle = -MAX_ROTATION_TICKS;
        }
        double dist = targetAngle - curPos;
        if(abs(dist)<20){
            checker.setState(TURRET_ROTATED, true);
            rotationMotor.setPower(0);
        }
        else {
            checker.setState(TURRET_ROTATED, false);
            double targetVelocity = pow(abs(dist), 1.4) / 69 * dist / abs(dist) * (100);
            rotationMotor.setMode(RUN_USING_ENCODER);
            rotationMotor.setVelocity(targetVelocity-(rotationMotor.getVelocity()-targetVelocity)/3);
        }

    }

    public void TurretManualRotation (double rotation) {
        if((rotatePosition > MAX_ROTATION_TICKS && rotation > 0) || (rotatePosition< -MAX_ROTATION_TICKS && rotation < 0)) {
            rotationMotor.setPower(0);
        }
        else {
            rotationMotor.setPower(rotation/3);
        }
    }


    public void setVelocity(double target) {
        rotationMotor.setVelocity(target);
    }


    public double getAngle() {
        return rotationMotor.getCurrentPosition();
    }
}