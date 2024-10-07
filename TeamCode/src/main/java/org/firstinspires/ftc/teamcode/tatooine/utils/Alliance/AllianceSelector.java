package org.firstinspires.ftc.teamcode.tatooine.utils.Alliance;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.IOException;

@TeleOp(name = "Alliance Selector", group = "Utils")
public class AllianceSelector extends LinearOpMode {
    boolean isRed = false;
    String basePath = "/sdcard/FIRST/";

    @Override
    public void runOpMode() throws InterruptedException {
        //check if the file "red.txt" exists
        File redFile = new File(basePath + "red.txt");
        isRed = CheckAlliance.isRed();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("enable", "to change alliance");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            if (isRed) {
                //delete the file "red.txt"
                redFile.delete();
            } else {
                //create the file "red.txt"
                try {
                    redFile.createNewFile();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

        }

    }
}
