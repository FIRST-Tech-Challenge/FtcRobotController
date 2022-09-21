package org.firstinspires.ftc.teamcode.subsystems.arm;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch.MagneticLimitSwitchSubsystem;

import java.util.HashMap;
import java.util.Map;


public class ArmSubsystem extends SubsystemBase {

    MagneticLimitSwitchSubsystem magneticLimitSwitch;
    //Motor in Port 1, Rev Hub 2
    private DcMotorEx motorArm = null;
    private DcMotorEx.Direction direction;
    private double power = 0.0;
    private DcMotorEx.RunMode mode;

    //private int armCurrentPos = 0;
    private static final int MIN_TARGET_POSITION = 0;
    private int maxTargetPositon = 1200;
    private final Telemetry telemetry;

    private boolean armLimitSwitchFlag = false;


    Map<Integer, Integer> levels = new HashMap<>();



    public ArmSubsystem(final HardwareMap hwMap, final String deviceName, MagneticLimitSwitchSubsystem magneticLimitSwitchSubsystem, Telemetry telemetry){
        motorArm = hwMap.get(DcMotorEx.class, deviceName);
        magneticLimitSwitch = magneticLimitSwitchSubsystem;
        this.telemetry = telemetry;
    }

    public ArmSubsystem(final HardwareMap hwMap, final String deviceName, MagneticLimitSwitchSubsystem magneticLimitSwitchSubsystem, DcMotorEx.RunMode mode, Telemetry telemetry){
        motorArm = hwMap.get(DcMotorEx.class, deviceName);
        magneticLimitSwitch = magneticLimitSwitchSubsystem;
        setMode(mode);
        this.telemetry = telemetry;

    }

    public ArmSubsystem(final HardwareMap hwMap, final String deviceName, MagneticLimitSwitchSubsystem magneticLimitSwitchSubsystem, DcMotorEx.RunMode mode, HashMap levels, Telemetry telemetry){
        motorArm = hwMap.get(DcMotorEx.class, deviceName);
        magneticLimitSwitch = magneticLimitSwitchSubsystem;
        setMode(mode);
        setLevels(levels);
        this.telemetry = telemetry;

    }

    public void setDirection(DcMotorEx.Direction direction){
        this.direction = direction;
        motorArm.setDirection(this.direction);
    }

    public void setPower(Double power){
        this.power = power;
        motorArm.setPower(this.power);
    }

    public void setMode(DcMotorEx.RunMode mode){
        this.mode = mode;
        motorArm.setMode(this.mode);
    }

    public void setZero(){
        //telemetry.addData("ArmSubsystem set zero",magneticLimitSwitch.armLimitSwitchPressed());


        if(magneticLimitSwitch.armLimitSwitchPressed() && !getArmLimitSwitchFlag())
        {
            setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armLimitSwitchFlag = true;

            //telemetry.addData("ArmSubsystem","set 0 limit switch");
            telemetry.update();
        }
        else if(getCurrentPosition() > 10 && !magneticLimitSwitch.armLimitSwitchPressed()){
            armLimitSwitchFlag = false;
        }

    }

    public void setArmTargetPosition(int armTargetPosition){
        //telemetry.addData("can reset",canResetArmFlag);
        //telemetry.update();
        setPower(0.5);
        motorArm.setTargetPosition(armTargetPosition);




    }
    public int getCurrentPosition(){
        return motorArm.getCurrentPosition();
    }

    public void setArmMaxTargetPosition(int maxTargetPosition){
        this.maxTargetPositon = maxTargetPosition;
    }

    public void addLevell(Integer levelIndicator, Integer level){
        levels.put(levelIndicator, level);
    }

    public void setLevels (HashMap levels){
        this.levels = levels;
    }

    public Integer getLevel(Integer levelIndicator){
        return levels.get(levelIndicator);
    }

    public boolean getArmLimitSwitchFlag(){
        return armLimitSwitchFlag;
    }

}
