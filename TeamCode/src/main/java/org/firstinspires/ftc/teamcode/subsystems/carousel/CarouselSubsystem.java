package org.firstinspires.ftc.teamcode.subsystems.carousel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CarouselSubsystem extends SubsystemBase {

    //Motor in Port 1, Rev Hub 2
    private DcMotorEx motorCarpusel = null;
    private DcMotorSimple.Direction direction;
    private double power = 0.0;
    private DcMotor.RunMode mode;

    //private int armCurrentPos = 0;
    private int targetPostion = 0;




    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName){
        motorCarpusel = hwMap.get(DcMotorEx.class, deviceName);

    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction){
        motorCarpusel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);

    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction, Double power){
        motorCarpusel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
        setPower(power);

    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotor.RunMode mode){
        motorCarpusel = hwMap.get(DcMotorEx.class, deviceName);
        setMode(mode);

    }

    public void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
        motorCarpusel.setDirection(this.direction);
    }

    public void setPower(Double power){
        this.power = power;
        motorCarpusel.setPower(this.power);
    }

    public void setMode(DcMotor.RunMode mode){
        this.mode = mode;
        motorCarpusel.setMode(this.mode);
    }

    public void setArmTargetPosition(int armTargetPosition){
        motorCarpusel.setTargetPosition(armTargetPosition);
    }
    public int getCurrentPosition(){
        return motorCarpusel.getCurrentPosition();
    }


}
