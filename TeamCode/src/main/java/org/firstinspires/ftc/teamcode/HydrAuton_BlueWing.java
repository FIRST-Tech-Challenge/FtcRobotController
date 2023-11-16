package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_BlueWingJ", preselectTeleOp = "HyDrive")
public class HydrAuton_BlueWing extends HydrAuton_Wing {
    public HydrAuton_BlueWing() {
        setTrueForRed = false;
        setTrueForRiggingOnRight = false;
    }
}
