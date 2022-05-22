package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class Logger {
    File myObj = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner myReader;
    File logFile;
    FileWriter wFTCfile;
    FileWriter indexer;
    String data = "0";
    OpMode op;
    public Logger (OpMode opMode){
        op = opMode;
        {
            try {
                myReader = new Scanner(myObj);
                data = myReader.nextLine();
                logFile = new File("/storage/emulated/0/tmp/Log"+data+".csv");
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
        {
            try {
                if (myObj.createNewFile()) {
                    op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                    op.telemetry.update();
                } else {
                    myObj.delete();
                    myObj.createNewFile();
                    op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                    op.telemetry.update();
                }
                indexer = new FileWriter(myObj);
                char a = data.charAt(0);
                a++;
                if(a == '9'){
                    a='0';
                }
                indexer.write(a);
                indexer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }


        try {
            //Create File
            if (logFile.createNewFile()) {
                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                op.telemetry.update();
            } else {
                logFile.delete();
                logFile.createNewFile();
                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                op.telemetry.update();
            }
        } catch (IOException e) {
            new RuntimeException("create file: FAILED", e).printStackTrace();
        }
        {
            try {
                wFTCfile = new FileWriter(logFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
    public void log(String newLog){
        try {
            wFTCfile.write(newLog);
        }catch(IOException e){
            e.printStackTrace();
        }
        op.telemetry.addData("log", newLog);

    }
    public void closeLog(){
        try {
            wFTCfile.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }
}
