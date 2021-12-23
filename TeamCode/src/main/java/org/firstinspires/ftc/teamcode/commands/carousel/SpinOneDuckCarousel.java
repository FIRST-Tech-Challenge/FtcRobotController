package org.firstinspires.ftc.teamcode.commands.carousel;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;

public class SpinOneDuckCarousel extends CommandBase {

    private final CarouselSubsystem carouselSubsytem;

    private final Double power;

    private final int oneSpinTargetPosition = 1000;

    private Telemetry telemetry;


    public SpinOneDuckCarousel(CarouselSubsystem subsystem, Double power){
        carouselSubsytem = subsystem;
        this.power = power;
        addRequirements(subsystem);
    }

    public SpinOneDuckCarousel(CarouselSubsystem subsystem, Double power, Telemetry telemetry){
        carouselSubsytem = subsystem;
        this.power = power;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

        carouselSubsytem.setCarouselTargetPosition(oneSpinTargetPosition, power);

        //while( carouselSubsytem.getCarouselCurrentPosition() < oneSpinTargetPosition){
            // wait for the spin to end! Is this necessary or the correct way?
            //not the correct way, see comment in isFinished
        //}
        //carouselSubsytem.setPower(0.0);
        //carouselSubsytem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //carouselSubsytem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        carouselSubsytem.setPower(0.0);
        carouselSubsytem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselSubsytem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean isFinished(){

        //instead of doing the while loop, you would do
        return carouselSubsytem.getCarouselCurrentPosition() > oneSpinTargetPosition;
        //return true;
    }


}
