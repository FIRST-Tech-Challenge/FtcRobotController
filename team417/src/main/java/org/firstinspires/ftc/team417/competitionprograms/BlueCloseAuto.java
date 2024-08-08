package org.firstinspires.ftc.team417.competitionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417.baseprograms.BaseAutonomous;

@Autonomous(name = "Blue Close Auto ")

public class BlueCloseAuto extends BaseAutonomous {
    public void runOpMode() {
        runAuto(false, true);
    }
}
