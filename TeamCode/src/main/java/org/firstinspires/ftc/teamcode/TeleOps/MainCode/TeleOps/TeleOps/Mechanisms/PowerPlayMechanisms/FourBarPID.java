package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Mechanisms.PowerPlayMechanisms;

import com.arcrobotics.ftclib.command.CommandBase;

public class FourBarPID extends CommandBase {

    private SyntaxErrorMechanisms mechanisms;
    private double target;

    public FourBarPID(Mechanisms fb, double target){
        this.target = target;
    }
    public FourBarPID(SyntaxErrorMechanisms fb, double target){
        this.mechanisms = fb;
        this.target = target;
    }

    @Override
    public void initialize() {}
    @Override
    public void execute(){
        mechanisms.runPID(target);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(mechanisms.getPosition() - target) < 25;
    }
    @Override
    public void end(boolean interrupted){
        mechanisms.runManual(0, 0);
    }
}