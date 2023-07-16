package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name= "LogReset")
public class LogReset extends LinearOpMode {
    File myObj = new File("/storage/emulated/0/tmp/LogIndex.csv");
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
        char a = '0';
        indexer.write(a);
        indexer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        stop();
    }
}