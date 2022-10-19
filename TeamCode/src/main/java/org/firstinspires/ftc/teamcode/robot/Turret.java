package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    //8192 * 5.23 * 1.8 * 4
    static final double     COUNTS_PER_MOTOR_REV    = 8192;         // ticks at the motor shaft
    static final double     DRIVE_GEAR_REDUCTION    = 5.23;         // 5:1 gear reduction (slowing down)
    static final double     TURRET_GEAR_REDUCTION   = 72.0 / 10.0;  // 10-teeth gear to 72-teeth gear
    public final double     NUMBEROFTICKSREVOLUTION = COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION*TURRET_GEAR_REDUCTION; // 308477.952
    public final double     NUMBEROFTICKSPERDEGREE  = NUMBEROFTICKSREVOLUTION/360;
    public final double     DEFAULT_TURRET_POWER = 0.1;
    public final double     INITIAL_MOVE_LEFT_TURRET_POWER = 0.1;
    public final double     HARD_STOP_CURRENT_DRAW = 100;
    public final String     TURRET_LEFT_POSITION = "Left";
    public final String     TURRET_RIGHT_POSITION = "Right";
    public final String     TURRET_CENTER_POSITION = "Center";

    public Telemetry telemetry;
    public DcMotorEx turretMotor;

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        //getting turret motor from the hardware map
        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");
        turretMotor.setPower(DEFAULT_TURRET_POWER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization

        while(turretMotor.getCurrent(CurrentUnit.MILLIAMPS) < HARD_STOP_CURRENT_DRAW) {
            turretMotor.setPower(INITIAL_MOVE_LEFT_TURRET_POWER);
        }
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void moveTurret(int desiredAngle){
        turretMotor.setTargetPosition(convertAngleToTicks(desiredAngle));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setMotor(double power){
        turretMotor.setPower(power);
    }
    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }

    private int convertAngleToTicks(double degrees){
        return (int)(degrees * NUMBEROFTICKSPERDEGREE);
    }
    public void moveToPreset(String presetName, Lift lift){
        switch(presetName){
            case TURRET_LEFT_POSITION:
            {
                //where code for turning left goes
                if(!lift.isInClear()){
                    lift.getToClear();
                }
                moveTurret(0);
                break;
            }
            case TURRET_RIGHT_POSITION:
            {
                if(!lift.isInClear()){
                    lift.getToClear();
                }
                moveTurret(180);
                break;
            }
            case TURRET_CENTER_POSITION:
            {
                if(!lift.isInClear()){
                    lift.getToClear();
                }
                moveTurret(90);
                break;
            }

        }

    }

}
