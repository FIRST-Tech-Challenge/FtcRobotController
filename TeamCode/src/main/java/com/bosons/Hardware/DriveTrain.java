package com.bosons.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain{
    public DcMotor FLDrive = null;
    public DcMotor FRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BRDrive = null;
    public double MotorPower = 0.0;

    public DriveTrain(OpMode op,double motorPower) {
        MotorPower = motorPower;
        FLDrive = op.hardwareMap.get(DcMotor.class, "fl");
        FRDrive = op.hardwareMap.get(DcMotor.class, "fr");
        BLDrive = op.hardwareMap.get(DcMotor.class, "bl");
        BRDrive = op.hardwareMap.get(DcMotor.class, "br");


        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);//this one good
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RunToTarget(DcMotor Drive,int Counts){
        Drive.setTargetPosition(Drive.getCurrentPosition() + Counts);
        Drive.setPower(MotorPower);
        Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int CalcMotorRatio(double C,Double PPR){
        double CircPi = C * Math.PI;
        return((int) ((1 / CircPi) * (PPR)));
    }

    public void MoveToPos(double Dist){
        double mm = Dist * (25.4 / 1);
        int Counts = (int) (CalcMotorRatio(100,537.7) * mm);

        RunToTarget(FRDrive,Counts);
        RunToTarget(FLDrive,Counts);
        RunToTarget(BRDrive,Counts);
        RunToTarget(BLDrive,Counts);
    }

    public void DebugWheels(double Dist){
        double mm = Dist * (25.4 / 1);
        int Counts = (int) (CalcMotorRatio(100,537.7) * mm);


        RunToTarget(FRDrive,Counts);
        RunToTarget(FLDrive,Counts);
        RunToTarget(BRDrive,Counts);
        RunToTarget(BLDrive,Counts);
    }
    public void KinematicMove(double x, double y, double turn){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), MotorPower);
        double FLPower = (y + x + turn) / denominator;
        double BLPower = (y - x + turn) / denominator;
        double FRPower = (y - x - turn) / denominator;
        double BRPower = (y + x - turn) / denominator;

        FLDrive.setPower(FLPower);
        BLDrive.setPower(BLPower);
        FRDrive.setPower(FRPower);
        BRDrive.setPower(BRPower);
    }
    // Send calculated power to wheels

}