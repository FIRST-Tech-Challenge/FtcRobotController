package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;


public class  IntakeSubsystem extends SubsystemBase {

    //Motor in Port 1, Rev Hub 2
    private DcMotorEx motorIntake = null;
    private DcMotorSimple.Direction direction;
    private double power = 0.0;
    private DcMotor.RunMode mode;

    //private int armCurrentPos = 0;
    private int targetPostion = 0;




    public IntakeSubsystem(final HardwareMap hwMap, final String deviceName){
        motorIntake = hwMap.get(DcMotorEx.class, deviceName);

    }

    public IntakeSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction){
        motorIntake = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);

    }

    public IntakeSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction, Double power){
        motorIntake = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
        setPower(power);

    }

    public IntakeSubsystem(final HardwareMap hwMap, final String deviceName, DcMotor.RunMode mode){
        motorIntake = hwMap.get(DcMotorEx.class, deviceName);
        setMode(mode);

    }

    public void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
        motorIntake.setDirection(this.direction);
    }

    public void setPower(Double power){
        this.power = power;
        motorIntake.setPower(this.power);
    }

    public void setMode(DcMotor.RunMode mode){
        this.mode = mode;
        motorIntake.setMode(this.mode);
    }

    public void setArmTargetPosition(int armTargetPosition){
        motorIntake.setTargetPosition(armTargetPosition);
    }
    public int getCurrentPosition(){
        return motorIntake.getCurrentPosition();
    }


}
