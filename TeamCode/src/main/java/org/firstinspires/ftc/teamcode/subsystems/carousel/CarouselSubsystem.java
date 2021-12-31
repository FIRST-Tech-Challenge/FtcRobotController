package org.firstinspires.ftc.teamcode.subsystems.carousel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class CarouselSubsystem extends SubsystemBase {

    //Motor in Port 1, Rev Hub 2
    private DcMotorEx motorCarousel;
    private DcMotorSimple.Direction direction;
    private double power = 0.0;
    private DcMotor.RunMode mode;
    private final Telemetry telemetry;

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        this.telemetry = telemetry;
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction, Telemetry telemetry){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
        this.telemetry = telemetry;
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotorSimple.Direction direction, Double power, Telemetry telemetry){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setDirection(direction);
        setPower(power);
        this.telemetry = telemetry;
    }

    public CarouselSubsystem(final HardwareMap hwMap, final String deviceName, DcMotor.RunMode mode,  Double power, Telemetry telemetry){
        motorCarousel = hwMap.get(DcMotorEx.class, deviceName);
        setMode(mode);
        setPower(power);
        this.telemetry = telemetry;
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
    public void setTargetPosition(int carouselTargetPosition){
        motorCarousel.setTargetPosition(carouselTargetPosition);
    }

    public void resetAndSetCarouselTargetPosition(int carouselTargetPosition, double power){
        telemetry.addData("Subsystem Carousel Power", power);
        telemetry.addData("Subsystem Carousel Pos", carouselTargetPosition);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(carouselTargetPosition);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }

    public int getCarouselCurrentPosition(){
        return motorCarousel.getCurrentPosition();
    }


}
