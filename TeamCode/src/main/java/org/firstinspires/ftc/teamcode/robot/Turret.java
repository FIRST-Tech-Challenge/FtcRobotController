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
    static final double     COUNTS_PER_MOTOR_REV    = 8192;         // ticks at the motor shaft
    static final double     DRIVE_GEAR_REDUCTION    = 5.23;         // 5:1 gear reduction (slowing down)
    static final double     TURRET_GEAR_REDUCTION   = 72.0 / 10.0;  // 10-teeth gear to 72-teeth gear
    public final double     NUMBEROFTICKSREVOLUTION = COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION*TURRET_GEAR_REDUCTION; // 308477.952
    public final double     NUMBEROFTICKSPERDEGREE  = NUMBEROFTICKSREVOLUTION/360;

    public Telemetry telemetry;
    public DcMotorEx turretMotor;

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        //getting turret motor from the hardware map
        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");
        turretMotor.setPower(0.5);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization
        turretMotor.setTargetPosition(30);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void moveTurret(int degree){
        int numberOfTicks = (int) (degree * NUMBEROFTICKSPERDEGREE);
        turretMotor.setTargetPosition(numberOfTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setMotor(double power){
        turretMotor.setPower(power);
    }

}
