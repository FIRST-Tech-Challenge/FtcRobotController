package org.firstinspires.ftc.teamcode.fishlo.program;

import org.firstinspires.ftc.teamcode.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.opMode.annotation.Program;
import org.firstinspires.ftc.teamcode.robot.Robot;


@Program(name = "Teleop", enabled = true)
public class TeleOp extends DriverControlledProgram {
    private Fishlo fishlo;

    @Override
    protected Robot buildRobot() {
        fishlo = new Fishlo(this);

        return fishlo;
    }
}
