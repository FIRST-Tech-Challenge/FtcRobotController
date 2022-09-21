package org.firstinspires.ftc.teamcode.commands.carousel;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;

public class MoveCarouselToPosition extends CommandBase {

    private final CarouselSubsystem carouselSubsytem;

    private final Double power;
    private final int postion;

    private Telemetry telemetry;


    public MoveCarouselToPosition(CarouselSubsystem subsystem, int position, Double power){
        carouselSubsytem = subsystem;
        this.postion = position;
        this.power = power;

        addRequirements(subsystem);
    }

    public MoveCarouselToPosition(CarouselSubsystem subsystem, int postion, Double power, Telemetry telemetry){
        carouselSubsytem = subsystem;
        this.postion = postion;
        this.power = power;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        carouselSubsytem.resetAndSetCarouselTargetPosition(postion,power);

        // telemetry.addData("MoveToCarouselPosition Initialize", carouselSubsytem.getCarouselCurrentPosition());
        //telemetry.update();
    }
    
    @Override
    public boolean isFinished(){

        //instead of doing the while loop, you would do
        return carouselSubsytem.getCarouselCurrentPosition() > postion;
        //return true;
    }

}
