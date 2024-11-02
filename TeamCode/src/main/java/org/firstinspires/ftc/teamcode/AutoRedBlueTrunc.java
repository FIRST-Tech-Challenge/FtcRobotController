package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red/Blue Truncated")
public class AutoRedBlueTrunc extends AutoCommandOpMode {
    public void logic() {
        schedule(new SequentialCommandGroup(
                drive(12)
                , turnCW(90)
                , drive(12)
                , turnCCW(90)
                /* do something with the arm*/
                , drive(-2)
                , turnCCW(90)
                , drive(12)
                , turnCW(90)
                , drive(12)
        ));
    }
}