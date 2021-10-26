package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;

public class CarouselDriveForward extends CommandBase {
    private Carousel m_carousel;
    private Telemetry m_telemetry;

    public CarouselDriveForward(Carousel carousel, Telemetry telemetry) {
        m_carousel = carousel;
        m_telemetry = telemetry;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() { m_carousel.drive(0.5); }

    @Override
    public boolean isFinished() { return false; }
}
