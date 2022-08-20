package org.firstinspires.ftc.teamcode.Components;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class Logger {
    File myFile = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner myReader;


    HashMap<String, File> logList = new HashMap<>();

    FileWriter indexer;
    FileWriter filewriter = null;
    String  currentTime;
    double runtime;
    double lastlogtime = 0;
    public int loopcounter=0;
    String data = "0";
    OpMode op;
    public Logger (OpMode opMode){
        op = opMode;

        try {
            myReader = new Scanner(myFile);
            data = myReader.nextLine();

//            if (myFile.createNewFile()) {
//                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
//                op.telemetry.update();
//            } else {
//                myFile.delete();
//                myFile.createNewFile();
//                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
//                op.telemetry.update();
//            }
            indexer = new FileWriter(myFile);
            char a = data.charAt(0);
            a++;
            if(a == '9'){
                a='0';
            }
            indexer.write(a);
            indexer.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void createFile (String fileName, String headers) {
        File file = new File("/storage/emulated/0/tmp/"+fileName+data+"Log.csv");
        try {
//            myReader = new Scanner(myFile);
//            data = myReader.nextLine();
//

            if (file.createNewFile()) {
                op.telemetry.addData("Logger:", "File created:%S\n", "Logger");
                op.telemetry.update();
            } else {
                file.delete();
                file.createNewFile();
                op.telemetry.addData("Logger:", "File already exists:%S\n", "Overriding");
                op.telemetry.update();
            }

            filewriter = new FileWriter(file, true);
            logList.put(fileName, file);
            currentTime = Calendar.getInstance().getTime().toString();

            filewriter.write(currentTime + "\n");
            filewriter.write(headers + "\n");
            filewriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }


    }

//    public void createFile(String newLogName) {
//        public static File newLogName;
//    }
    @SuppressLint("DefaultLocale")
    public void log(String fileName, String input){
//        if (loopcounter % 5 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
//        }
    }

    @SuppressLint("DefaultLocale")
    public void logRegulated(String fileName, String input){
        if (loopcounter % 10 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }




    @SuppressLint("DefaultLocale")
    public void log(String fileName, double input){
        String inputstringversion = String.valueOf(input);
        if (loopcounter % 30 == 0) {

            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, int input){
        String inputstringversion = String.valueOf(input);
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, float input){
        String inputstringversion = String.valueOf(input);
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, boolean input){
        String inputstringversion = String.valueOf(input);
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + "," + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    public void closeLog(){
//        try {
//            Collection<FileWriter> files = logList.values();
//            for (FileWriter file: files
//                 ) {
//                file.close();
//            }
//        }
//        catch(IOException e){
//            e.printStackTrace();
//        }


    }
}
