package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuton_BlueBackstageJ", preselectTeleOp = "HyDrive")
public class HydrAuton_BlueBackstage extends HydrAuton_Backstage {
    public HydrAuton_BlueBackstage() {
        setTrueForRed = false;
        setTrueForRiggingOnRight = true;
        modelFilename = "Blue_Prop.tflite";
    }
}
