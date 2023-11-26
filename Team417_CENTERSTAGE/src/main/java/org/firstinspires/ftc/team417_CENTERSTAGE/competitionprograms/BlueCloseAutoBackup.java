package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.League1BaseAutonomous;

@Autonomous (name="Blue Close Auto Backup ")

public class BlueCloseAutoBackup extends League1BaseAutonomous {
    public void runOpMode() {
        runSimpleInchesAuto(false, true);

        // move forward from the wall
    }
}
