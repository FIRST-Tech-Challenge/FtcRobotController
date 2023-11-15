package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_BlueBackdrop", preselectTeleOp = "HyDrive")
public class HydrAuton_BlueBackdrop extends HydrAuton_Backdrop {
    public HydrAuton_BlueBackdrop () {
        setTrueForRed = false;
        setTrueForRiggingOnRight = true;
    }
}
