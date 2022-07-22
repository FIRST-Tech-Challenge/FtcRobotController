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
    public static File odometryFile;
    FileWriter wFTCodometryfile;

    public static File sequencingFile;
    FileWriter wFTCsequencingfile;

    public static File miscFile;
    FileWriter wFTCmiscfile;

    FileWriter indexer;
    String data = "0";
    OpMode op;
    public Logger (OpMode opMode){
        op = opMode;
        {
            try {
                myReader = new Scanner(myObj);
                data = myReader.nextLine();
                odometryFile = new File("/storage/emulated/0/tmp/Log"+data+".csv");
                sequencingFile = new File("/storage/emulated/0/tmp/Log"+data+".csv");
                miscFile = new File("/storage/emulated/0/tmp/Log"+data+".csv");
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
            if (odometryFile.createNewFile()) {
                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                op.telemetry.update();
            } else {
                odometryFile.delete();
                odometryFile.createNewFile();
                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                op.telemetry.update();
            }

            if (sequencingFile.createNewFile()) {
                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                op.telemetry.update();
            } else {
                sequencingFile.delete();
                sequencingFile.createNewFile();
                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                op.telemetry.update();
            }

            if (miscFile.createNewFile()) {
                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                op.telemetry.update();
            } else {
                miscFile.delete();
                miscFile.createNewFile();
                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                op.telemetry.update();
            }

        } catch (IOException e) {
            new RuntimeException("create file: FAILED", e).printStackTrace();
        }

        {
            try {
                wFTCodometryfile = new FileWriter(odometryFile);
                wFTCsequencingfile = new FileWriter(sequencingFile);
                wFTCmiscfile = new FileWriter(miscFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
    public void log(String newLog, File logFile){
        try {
            if (logFile == odometryFile) {
                wFTCodometryfile.write(newLog);
            }
            else if (logFile == sequencingFile) {
                wFTCsequencingfile.write(newLog);
            }
            else {
                wFTCmiscfile.write(newLog);
            }
        }catch(IOException e){
            e.printStackTrace();
        }

    }
    public void closeLog(File logFile){
        try {
            if (logFile == odometryFile) {
                wFTCodometryfile.close();
            }
            else if (logFile == sequencingFile) {
                wFTCsequencingfile.close();
            }
            else {
                wFTCmiscfile.close();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }
}
