package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Blue Close Auto Backup ")

public class BlueCloseAutoBackup extends BaseAutonomous {
    public void runOpMode() {
        runSimpleInchesAuto(false, true);

        // move forward from the wall
    }
}
