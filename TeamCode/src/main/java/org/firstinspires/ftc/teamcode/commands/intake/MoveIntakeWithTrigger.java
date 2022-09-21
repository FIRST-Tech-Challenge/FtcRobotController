package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MoveIntakeWithTrigger extends CommandBase {

    private final IntakeSubsystem intakeSubsytem;

    private final double triggerPower;

    private boolean triggerStopped = false;
    private static final double ZERO_POWER = 0.0;
    private static final double TIMEOUT = 800;
    private static final double TRIGGER_ACTIVE_THRESHOLD = 0.5;

    //private ElapsedTime intakeTimer = new ElapsedTime();;

    private Telemetry telemetry;


    public MoveIntakeWithTrigger(IntakeSubsystem subsystem, final double power){
        intakeSubsytem = subsystem;
        triggerPower = power;

        addRequirements(subsystem);
    }

    public MoveIntakeWithTrigger(IntakeSubsystem subsystem, final double power, Telemetry telemetry){

        //telemetry.addData("make trigger:", power );
        //telemetry.update();

        intakeSubsytem = subsystem;
        triggerPower = power;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        //intakeTimer.reset();
        intakeSubsytem.setPower(triggerPower);
    }

    public void end(){
        //intakeSubsytem.setPower(ZERO_POWER);
    }


    @Override
    public boolean isFinished(){
        return true;
    }
    /*public boolean isFinished(){
        /*
        while(intakeTimer.milliseconds() < TIMEOUT){
            telemetry.addData("intake complete", intakeTimer.milliseconds());
            telemetry.update();
        }

        return intakeTimer.milliseconds() > TIMEOUT;
    }*/


}
