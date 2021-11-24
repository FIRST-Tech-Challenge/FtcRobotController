package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

public class MLTest extends AutonomousTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initAll();
        getClass().getResource("Trained Pink Team Marker Finder.tflite");
    }
}
