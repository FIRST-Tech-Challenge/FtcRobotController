package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_RedWing", preselectTeleOp = "HyDrive")
public class HydrAuton_RedWing extends HydrAuton_Wing {
    public HydrAuton_RedWing() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = true;
    }
}
