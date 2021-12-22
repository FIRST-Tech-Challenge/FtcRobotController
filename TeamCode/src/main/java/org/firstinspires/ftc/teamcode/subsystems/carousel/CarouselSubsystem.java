package org.firstinspires.ftc.teamcode.subsystems.carousel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CarouselSubsystem extends SubsystemBase {

    //Motor in Port 1, Rev Hub 2
    private DcMotorEx motorCarousel = null;
    private DcMotorSimple.Direction direction;
    private double power = 0.0;
    private DcMotor.RunMode mode;

    //private int armCurrentPos = 0;
    private int targetPostion = 0;


    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction, Double power){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
        setPower(power);
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotor.RunMode mode){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setMode(mode);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
        motorCarousel.setDirection(this.direction);
    }

    public void setPower(Double power){
        this.power = power;
        motorCarousel.setPower(this.power);
    }

    public void setMode(DcMotor.RunMode mode){
        this.mode = mode;
        motorCarousel.setMode(this.mode);
    }

    public void setArmTargetPosition(int armTargetPosition){
        motorCarousel.setTargetPosition(armTargetPosition);
    }

    public int getCurrentPosition(){
        return motorCarousel.getCurrentPosition();
    }


}
