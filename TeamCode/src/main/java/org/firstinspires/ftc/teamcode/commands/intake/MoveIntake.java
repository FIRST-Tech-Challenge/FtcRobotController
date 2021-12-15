package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class MoveIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsytem;

    private final double power;
    private final DoubleSupplier lTrigger;
    private final DoubleSupplier rTrigger;
    private boolean triggerStopped = false;

    private Telemetry telemetry;


    public MoveIntake(IntakeSubsystem subsystem, double power, DoubleSupplier lt, DoubleSupplier rt){
        m_intakeSubsytem = subsystem;
        this.power = power;
        lTrigger = lt;
        rTrigger = rt;
        addRequirements(subsystem);
    }

    public MoveIntake(IntakeSubsystem subsystem, double power, DoubleSupplier lt, DoubleSupplier rt, Telemetry telemetry){
        m_intakeSubsytem = subsystem;
        this.power = power;
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
        if(lTrigger.getAsDouble() > 0.5) {
            m_intakeSubsytem.setPower(0.6);
            triggerStopped = false;
        }
        else if(rTrigger.getAsDouble() > 0.5) {
            m_intakeSubsytem.setPower(-0.75);
            triggerStopped = false;
        }

        else{
            m_intakeSubsytem.setPower(0.0);
            triggerStopped = true;
        }

    }

    /*@Override
    public boolean isFinished(){
        //return triggerStopped;
    }*/



}
