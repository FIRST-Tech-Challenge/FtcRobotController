package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class StopIntake extends CommandBase {

    private final IntakeSubsystem intakeSubsytem;

    private static final Double ZERO_POWER = 0.0;

    private Telemetry telemetry;


    public StopIntake(IntakeSubsystem subsystem){
        intakeSubsytem = subsystem;
        addRequirements(subsystem);
    }

    public StopIntake(IntakeSubsystem subsystem, Telemetry telemetry){
        intakeSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        //telemetry.addData("Stopping intake in command", ZERO_POWER);
        intakeSubsytem.setPower(ZERO_POWER);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
