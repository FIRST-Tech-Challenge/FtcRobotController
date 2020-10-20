package org.firstinspires.ftc.teamcode.fishlo.program;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;



@TeleOp()
public class Teleop extends DriverControlledProgram {
    private Fishlo fishlo;

    @Override
    protected Robot buildRobot() {
        fishlo = new Fishlo(this);

        return fishlo;
    }
}
