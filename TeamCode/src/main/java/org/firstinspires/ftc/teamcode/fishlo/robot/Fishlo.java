package org.firstinspires.ftc.teamcode.fishlo.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Fishlo extends Robot {

    public Fishlo(OpMode opMode) {
        super(opMode);

        putSubSystem("Drive", new Drive(this));
        putSubSystem("Claw", new Claw(this));
        putSubSystem("Lift", new Lift(this));
    }

}
