package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.commands.ArmHighGoal;
import org.firstinspires.ftc.teamcode.commands.IntakeCmd;

@Autonomous(name = "Auto Red/Blue")
public class AutoRedBlue extends AutoCommandOpMode {
    public void logic() {
        schedule(new SequentialCommandGroup(
                /*step 1: go to central position with PinkAqua*/
                drive(12)
                , turnCW(90)
                , drive(9)
                , turnCCW(90)

                /*step 2: go to cage (this has a different name)*/
                , drive(16)
                /* do something with the arm*/

                /*step 3: go to board*/
                , drive(-9)
                , turnCCW(90)
                , drive(40)
                , turnCCW(45)
                , drive(12)
                /*do something with the arm*/

                /*step 4: park*/
                , drive(-30)
                , turnCCW(45)
                , drive(-36)
                , turnCCW(70)
                , drive(24)
        ));
    }
}