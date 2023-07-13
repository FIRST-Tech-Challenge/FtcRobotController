package org.firstinspires.ftc.teamcode.Tests;


import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name= "PosLogReset")
public class PosLogReset extends LinearOpMode {
    @SuppressLint("SdCardPath")
    File myObj = new File("/sdcard/tmp/pos.csv");
    FileWriter indexer;

    public void runOpMode() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        try{
            if (myObj.createNewFile()) {
                telemetry.addData("Logger:", "File created:%S\n", "Logger");
                telemetry.update();
            } else {
                myObj.delete();
                myObj.createNewFile();
                telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                telemetry.update();
            }
            indexer = new FileWriter(myObj);
            indexer.write(0+"\n"+0+"\n"+0);
            indexer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        stop();
    }
}