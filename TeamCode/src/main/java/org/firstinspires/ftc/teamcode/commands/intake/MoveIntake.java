package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class MoveIntake extends CommandBase {

    private final IntakeSubsystem intakeSubsytem;

    private final double rTriggerPower;
    private final double lTriggerPower;
    private final DoubleSupplier lTrigger;
    private final DoubleSupplier rTrigger;
    private boolean triggerStopped = false;
    private static final double ZERO_POWER = 0.0;
    private static final double TRIGGER_ACTIVE_THRESHOLD = 0.5;

    private Telemetry telemetry;


    public MoveIntake(IntakeSubsystem subsystem, final double rPower, final double lPower, DoubleSupplier lt, DoubleSupplier rt){
        intakeSubsytem = subsystem;
        rTriggerPower = rPower;
        lTriggerPower = lPower;
        lTrigger = lt;
        rTrigger = rt;
        addRequirements(subsystem);
    }

    public MoveIntake(IntakeSubsystem subsystem, final double rPower, final double lPower, DoubleSupplier lt, DoubleSupplier rt, Telemetry telemetry){
        intakeSubsytem = subsystem;
        rTriggerPower = rPower;
        lTriggerPower = lPower;
        lTrigger = lt;
        rTrigger = rt;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        telemetry.addData("Move intake lt command",lTrigger.getAsDouble());
        telemetry.addData("Move intake lt command",rTrigger.getAsDouble());

        telemetry.update();
        if(lTrigger.getAsDouble() > TRIGGER_ACTIVE_THRESHOLD) {
            intakeSubsytem.setPower(lTriggerPower);
            triggerStopped = false;
        }
        else if(rTrigger.getAsDouble() > TRIGGER_ACTIVE_THRESHOLD) {
            intakeSubsytem.setPower(rTriggerPower);
            triggerStopped = false;
        }

        else{
            intakeSubsytem.setPower(ZERO_POWER);
            triggerStopped = true;
        }

    }

    /*@Override
    public boolean isFinished(){
        //return triggerStopped;
    }*/



}
