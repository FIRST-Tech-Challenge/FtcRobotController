package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Scanner;

public class Logger {
    File LogIndex = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner logindexReader;

    HashMap<String, File> logList = new HashMap<>();

    HashMap<File, ArrayList<Integer>> headerList = new HashMap<>();

    ArrayList<Integer> tempheaderPositions = new ArrayList<>();

    ArrayList<String> inputStrings = new ArrayList<>();

    FileWriter logindexer;
    FileWriter filewriter = null;
    String currentTime;
    public int loopcounter=0;
    String data = "0";
    String loggingString;
    char prevchar;
    int previnputstringstartindex = 0;

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
    public void logPos(Pose2d position){
        try {
            FileWriter poser = new FileWriter("/sdcard/tmp/pos.csv");
            poser.write(position.getX()+"\n"+position.getY()+"\n"+position.getHeading());
            poser.close();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public Pose2d readLogPos(){
        Pose2d returner = new Pose2d(0,0,0);
        try {
            Scanner poser = new Scanner("/storage/emulated/0/tmp/pos.csv");
            double[] come = {0,0,0};
            for(int i=0;i<3;i++){
                if(poser.hasNextLine()){
                    come[i]=Double.parseDouble(poser.nextLine());
                }
            }
            poser.close();
            returner = new Pose2d(come[0],come[1],come[2]);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return returner;
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
    public void log(String fileName, String input) {
        try {
            File file = new File("/sdcard/tmp/"+fileName+data+"Log.csv");
            FileWriter filewriter = new FileWriter(file, true);
            filewriter.write(String.format("%.2f", time) + ":" + input + "\n");
            filewriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @SuppressLint("DefaultLocale")
    public void log(String fileName, String input, boolean p_isFormatted){
        if (p_isFormatted) {
            loggingString = "";
            tempheaderPositions = headerList.get(logList.get(fileName));
            previnputstringstartindex = 0;
            inputStrings.clear();

            for (int i = 1; i < input.length(); i++) {
                prevchar = input.charAt(i - 1);
                if (prevchar == ',') {
                    inputStrings.add(input.substring(previnputstringstartindex, i - 1));
                    previnputstringstartindex = i;
                }
            }

            inputStrings.add(input.substring(previnputstringstartindex));

            for (int i = 0; i < inputStrings.size(); i++) {

                while (tempheaderPositions.get(i + 1)- loggingString.length()-String.format("%.2f", time).length() - 1 >0) {
                    loggingString += " ";
                }
                loggingString += inputStrings.get(i);
            }

            try {
                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                filewriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                filewriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        else {
            log(fileName, input);
        }
    }
    @SuppressLint("DefaultLocale")
    public void logNoTime(String fileName, String input){
        try {
            File file = new File("/sdcard/tmp/"+fileName+data+"Log.csv");
            FileWriter filewriter = new FileWriter(file, true);
            filewriter.write(input + "\n");
            filewriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, String input, boolean p_isFormatted, boolean p_isRegulated){
        if (p_isFormatted && p_isRegulated) {
            loggingString = "";
            tempheaderPositions = headerList.get(logList.get(fileName));
            previnputstringstartindex = 0;
            inputStrings.clear();

            if (loopcounter % 20 == 0) {

                for (int i = 1; i < input.length(); i++) {
                    prevchar = input.charAt(i - 1);
                    if (prevchar == ',') {
                        inputStrings.add(input.substring(previnputstringstartindex, i - 1));
                        previnputstringstartindex = i;
                    }
                }

                inputStrings.add(input.substring(previnputstringstartindex));

                for (int i = 0; i < inputStrings.size(); i++) {

                    while (tempheaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", time).length() - 1 > 0) {
                        loggingString += " ";
                    }
                    loggingString += inputStrings.get(i);
                }

                try {
                    FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                    filewriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                    filewriter.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }


        }

        else if (p_isFormatted && !p_isRegulated) {
            log(fileName, input, p_isFormatted);
        }

        else if (!p_isFormatted && p_isRegulated) {
            if (loopcounter % 20 == 0) {
                log(fileName, input);
            }
        }

        else {
            log(fileName, input);
        }
    }

    @SuppressLint("DefaultLocale")
    public void log(String fileName, String input, boolean p_isFormatted, boolean p_isRegulated, boolean p_isTelemetry){
        if (p_isFormatted && p_isRegulated) {
            loggingString = "";
            tempheaderPositions = headerList.get(logList.get(fileName));
            previnputstringstartindex = 0;
            inputStrings.clear();

            if (loopcounter % 20 == 0) {

                for (int i = 1; i < input.length(); i++) {
                    prevchar = input.charAt(i - 1);
                    if (prevchar == ',') {
                        inputStrings.add(input.substring(previnputstringstartindex, i - 1));
                        previnputstringstartindex = i;
                    }
                }

                inputStrings.add(input.substring(previnputstringstartindex));

                for (int i = 0; i < inputStrings.size(); i++) {

                    while (tempheaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", time).length() - 1 > 0) {
                        loggingString += " ";
                    }
                    loggingString += inputStrings.get(i);
                }

                try {
                    FileWriter filewriter = new FileWriter(logList.get(fileName), true);
                    filewriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                    filewriter.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }


        }

        else if (p_isFormatted && !p_isRegulated) {
            log(fileName, input, p_isFormatted);
        }

        else if (!p_isFormatted && p_isRegulated) {
            if (loopcounter % 20 == 0) {
                log(fileName, input);
            }
        }

        else if (!p_isFormatted && !p_isRegulated && p_isTelemetry) {
            log(fileName, input);
            op.telemetry.addData("Log Telemetry: ", input);
        }

        else {
            log(fileName, input);
        }
    }

//    public void logMessage(String fileName, String input){
////        if (loopcounter % 5 == 0) {
//        try {
//            FileWriter filewriter = new FileWriter(logList.get(fileName), true);
//            filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
//            filewriter.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
////        }
//    }

//    @SuppressLint("DefaultLocale")
//    public void logRegulated(String fileName, String input){
//
//        loggingString = "";
//        tempheaderPositions = headerList.get(logList.get(fileName));
//        previnputstringstartindex = 0;
//        inputStrings.clear();
//
//        if (loopcounter % 20 == 0) {
//
//            for (int i = 1; i < input.length(); i++) {
//                prevchar = input.charAt(i - 1);
//                if (prevchar == ',') {
//                    inputStrings.add(input.substring(previnputstringstartindex, i - 1));
//                    previnputstringstartindex = i;
//                }
//            }
//
//            inputStrings.add(input.substring(previnputstringstartindex));
//
//            for (int i = 0; i < inputStrings.size(); i++) {
//
//                while (tempheaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", op.getRuntime()).length() - 1 > 0) {
//                    loggingString += " ";
//                }
//                loggingString += inputStrings.get(i);
//            }
//
//            try {
//                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
//                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + loggingString + "\n");
//                filewriter.close();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//
//        }
//    }
//
//    public void logRegulatedMessage(String fileName, String input) {
//        if (loopcounter % 10 == 0) {
//            try {
//                FileWriter filewriter = new FileWriter(logList.get(fileName), true);
//                filewriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
//                filewriter.close();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }

    public void closeLog(){
        File[] fileArray = logList.values().toArray(new File[1]);
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
