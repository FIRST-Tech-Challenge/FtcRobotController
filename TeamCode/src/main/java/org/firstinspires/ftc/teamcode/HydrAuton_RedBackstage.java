package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_RedBackstageJ", preselectTeleOp = "HyDrive")
public class HydrAuton_RedBackstage extends HydrAuton_Backdrop {
    public HydrAuton_RedBackstage() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = false;
    }
}
