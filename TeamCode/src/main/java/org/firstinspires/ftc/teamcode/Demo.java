package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmHighGoal;

@Autonomous(name = "Demo")
public class Demo extends AutoCommandOpMode {
    public void logic(){
        schedule(new SequentialCommandGroup(
                armHighGoal
        ));
    }
}