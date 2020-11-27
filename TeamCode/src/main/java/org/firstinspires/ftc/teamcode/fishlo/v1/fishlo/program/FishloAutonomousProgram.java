package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.*;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {
    protected Claw claw;
    protected Drive drive;
    protected Gyro gyro;
    protected Vision vision;
    protected Intake intake;
    protected Shooter shooter;

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);

        drive = (Drive) fishlo.getSubSystem("Drive");
        claw = (Claw) fishlo.getSubSystem("Claw");
        gyro = (Gyro) fishlo.getSubSystem("Gyro");
        vision = (Vision) fishlo.getSubSystem("Vision");
        intake = (Intake) fishlo.getSubSystem("Intake");
        shooter = (Shooter) fishlo.getSubSystem("Shooter");

        return fishlo;
    }

    @Override
    public void main() {}

    @Override
    public void preMain() {}
}
