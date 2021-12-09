package org.firstinspires.ftc.teamcode.commands.carousel;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;

public class MoveCarousel extends CommandBase {

    private final CarouselSubsystem m_carouselSubsytem;

    private final Double power;

    private Telemetry telemetry;


    public MoveCarousel(CarouselSubsystem subsystem, Double power){
        m_carouselSubsytem = subsystem;
        this.power = power;
        addRequirements(subsystem);
    }

    public MoveCarousel(CarouselSubsystem subsystem, Double power, Telemetry telemetry){
        m_carouselSubsytem = subsystem;
        this.power = power;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        m_carouselSubsytem.setPower(power);
    }



}
