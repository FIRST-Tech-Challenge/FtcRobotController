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
    public int getPosition (){
        int position = turretMotor.getCurrentPosition();
        return position;
    }

}
