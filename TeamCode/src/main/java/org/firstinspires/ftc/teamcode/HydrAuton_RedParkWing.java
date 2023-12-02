package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_RedParkWingJ", preselectTeleOp = "HyDrive")
public class HydrAuton_RedParkWing extends HydrAuton_WingParkOnly{
    public HydrAuton_RedParkWing() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = true;
        modelFilename = "Red_Prop.tflite";
        mOpModeName = "Red-Wing-Park";
        mWaitTimeAtRigging = 18000;
    }
}
