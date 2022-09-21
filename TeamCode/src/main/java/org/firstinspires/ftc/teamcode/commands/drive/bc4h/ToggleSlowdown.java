package org.firstinspires.ftc.teamcode.commands.drive.bc4h;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.bc4h.BC4HDriveSubsystem;

public class ToggleSlowdown extends CommandBase {

    private final BC4HDriveSubsystem drive;


    private Telemetry telemetry;


    public ToggleSlowdown(BC4HDriveSubsystem subsystem){
        drive = subsystem;
        addRequirements(subsystem);
    }

    public ToggleSlowdown(BC4HDriveSubsystem subsystem, Telemetry telemetry){
        drive = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        telemetry.addData("SlowdownFlag", drive.getSlowdownFlag());
        drive.setSlowdownFlag(!drive.getSlowdownFlag());

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }


}
