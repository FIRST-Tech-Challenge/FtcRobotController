package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MoveIntakeWithTrigger extends CommandBase {

    private final IntakeSubsystem intakeSubsytem;

    private final double triggerPower;
    private final DoubleSupplier trigger;

    private boolean triggerStopped = false;
    private static final double ZERO_POWER = 0.0;
    private static final double TRIGGER_ACTIVE_THRESHOLD = 0.5;

    private Telemetry telemetry;


    public MoveIntakeWithTrigger(IntakeSubsystem subsystem, final double power, DoubleSupplier trig){
        intakeSubsytem = subsystem;
        triggerPower = power;
        trigger = trig;

        addRequirements(subsystem);
    }

    public MoveIntakeWithTrigger(IntakeSubsystem subsystem, final double power, DoubleSupplier trig, Telemetry telemetry){

        telemetry.addData("make trigger:", power );
        //telemetry.update();

        intakeSubsytem = subsystem;
        triggerPower = power;
        trigger = trig;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        telemetry.addData("trigger", TRIGGER_ACTIVE_THRESHOLD );
        telemetry.update();
        if(trigger.getAsDouble() > TRIGGER_ACTIVE_THRESHOLD) {
            intakeSubsytem.setPower(triggerPower);
            triggerStopped = false;
        }
        else{
            intakeSubsytem.setPower(ZERO_POWER);
            triggerStopped = true;
        }

    }

    @Override
    public boolean isFinished(){
        schedule(true);
        telemetry.addData("trigger done", TRIGGER_ACTIVE_THRESHOLD );
        telemetry.update();
        return Thread.currentThread().isInterrupted() || triggerStopped;

    }



}
