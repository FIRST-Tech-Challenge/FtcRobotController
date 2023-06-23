package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

public abstract class PoofyFeedForwardController {

    public abstract double calculate(double measuredPosition);

    public abstract void setTargetPosition(double targetPosition);

}
