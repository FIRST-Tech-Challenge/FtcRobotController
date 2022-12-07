package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Lift_SS;

public class LiftDown_CMD extends CommandBase {

    private final Lift_SS  mLift_SS;
    private final Telemetry mTelemetry;

    private final Gamepad mGamepad;
    private  double liftSpeed;

    public LiftDown_CMD(Telemetry telemetry, Lift_SS p_Lift_SS,Gamepad gamepad, double p_Speed ){
        mTelemetry = telemetry;
        mLift_SS  = p_Lift_SS;
        mGamepad = gamepad;


        addRequirements(p_Lift_SS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute( ) {

        double downSpeed= -liftSpeed;
        mLift_SS.liftDown(downSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mLift_SS.stopLift();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
