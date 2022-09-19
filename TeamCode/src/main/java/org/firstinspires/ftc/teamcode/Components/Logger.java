package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collection;
import java.util.HashMap;
import java.util.Scanner;

public class Logger {
    File LogIndex = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner logindexReader;

    HashMap<String, File> logList = new HashMap<>();

    HashMap<File, ArrayList<Integer>> headerList = new HashMap<>();

    ArrayList<Integer> tempheaderPositions = new ArrayList<>();

    FileWriter logindexer;
    FileWriter filewriter = null;
    String currentTime;
    public int loopcounter=0;
    String data = "0";
    String loggingString;
    char prevchar;

    @SuppressLint("SdCardPath")
    public Logger (){
        new File("/sdcard/tmp/MotorLogs").mkdirs();
        new File("/sdcard/tmp/ServoLogs").mkdirs();
        new File("/sdcard/tmp/DualServoLogs").mkdirs();
        new File("/sdcard/tmp/CRServoLogs").mkdirs();
        new File("/sdcard/tmp/RobotLogs").mkdirs();
        try {
            logindexReader = new Scanner(LogIndex);
            data = logindexReader.nextLine();
            logindexer = new FileWriter(LogIndex);

            char a = data.charAt(0);
            a++;
            if(a == '9'){
                a='0';
            }

            logindexer.write(a);
            logindexer.close();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    @SuppressLint("SdCardPath")
    public void createFile (String fileName, String headers) {
        File file = new File("/sdcard/tmp/"+fileName+data+"Log.csv");
        headerList.computeIfAbsent(file, k -> new ArrayList<>());
        headerList.get(file).add(0);
        for (int i = 2; i < headers.length(); i++) {
            prevchar = headers.charAt(i - 1);
            if (prevchar == ' ' && headers.charAt(i) != prevchar) {
                headerList.get(file).add(i);
            }
        }

        try {
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
            for (int i = 0; i < headers.length(); i++) {
                filewriter.write("=");
            }
            filewriter.write("\n");
            filewriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, ArrayList<String> input){
        loggingString = "";
        tempheaderPositions = headerList.get(logList.get(fileName));
        for (int i = 0; i < input.size(); i++) {

            while (tempheaderPositions.get(i + 1)- loggingString.length()-String.format("%.2f", op.getRuntime()).length() - 1 >0) {
                loggingString += " ";
            }
            loggingString += input.get(i);
        }

            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + loggingString + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    public void logMessage(String fileName, String input){
//        if (loopcounter % 5 == 0) {
        try {
            FileWriter filewriter = new FileWriter(logList.get(fileName), true);
            filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
            filewriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
//        }
    }

    @SuppressLint("DefaultLocale")
    public void logRegulated(String fileName, ArrayList<String> input) {
        loggingString = "";
        tempheaderPositions = headerList.get(logList.get(fileName));
        for (int i = 0; i < input.size(); i++) {
            while (tempheaderPositions.get(i + 1)- loggingString.length()-String.format("%.2f", op.getRuntime()).length() - 1 > 0) {
                loggingString += " ";
            }
            loggingString += input.get(i);
        }

        if (loopcounter % 10 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + loggingString + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        loggingString = "";
    }

    public void logRegulatedMessage(String fileName, String input) {
        if (loopcounter % 10 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + loggingString + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }




    @SuppressLint("DefaultLocale")
    public void log(String fileName, double input){
        if (loopcounter % 30 == 0) {

            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, int input){
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, float input){
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, boolean input){
        if (loopcounter % 30 == 0) {
            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    public void closeLog(){
        Collection<File> fileCollection = logList.values();
        Object[] fileArrayList= fileCollection.toArray();
        File[] fileArray = (File[]) Arrays.stream(fileArrayList).toArray();
        try {
            for (int i=0; i< fileArray.length;i++){
                FileWriter file = new FileWriter(fileArray[i]);
                file.close();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }
}
