package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    //8192 * 5.23 * 1.8 * 4
    public final double NUMBEROFTICKSREVOLUTION =308477.952;
    public final double NUMBEROFTICKSPERDEGREE  = NUMBEROFTICKSREVOLUTION/360;
    public Telemetry telemetry;
    public DcMotorEx turretMotor;
    @SuppressLint("NotConstructor")
    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        //getting turret motor from the hardware map
        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization
        turretMotor.setTargetPosition(300);
        turretMotor.setPower(1);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveTurret(int targetDegrees){
        int numberOfTicks = (int) (targetDegrees * NUMBEROFTICKSPERDEGREE);
        turretMotor.setTargetPosition(numberOfTicks);
        turretMotor.setPower(0.5);
    }

}
