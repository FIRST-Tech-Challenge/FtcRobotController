package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Pink/Aqua")
public class AutoPinkAqua extends AutoCommandOpMode {
    @Override
    public void logic() {
        schedule(new SequentialCommandGroup(
                drive(12)
                , turnCCW(90)
                , drive(12)
                , turnCW(90)
                /* do something with the arm*/
                , drive(-2)
                , turnCW(90)
                , drive(12)
                , turnCCW(90)
                , drive(12)
        ));
    }
}