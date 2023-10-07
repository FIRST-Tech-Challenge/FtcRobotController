package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Objects;
import java.util.Scanner;

/**
 * Harry
 */
public class Logger {

    /**
     * This class is used to log information from any function in any class to a file in order to help the user debug
     *     * their code. There are various different options for logging; the functions themselves can be found below, and
     *     * examples can be found in motor and servo classes.
     */

    File logIndex = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner logIndexReader;

    HashMap<String, File> logList = new HashMap<>();

    HashMap<File, ArrayList<Integer>> headerList = new HashMap<>();

    ArrayList<Integer> tempHeaderPositions = new ArrayList<>();

    ArrayList<String> inputStrings = new ArrayList<>();

    FileWriter logIndexer;
    FileWriter fileWriter = null;
    String currentTime;
    String data = "0";
    StringBuilder loggingString;

    public int loopCounter=0;
    public static final DecimalFormat df = new DecimalFormat("0.00");
    char prevChar;
    int prevInputStringStartIndex = 0;

    /**
     * Constructor
     */

    @SuppressLint("SdCardPath")
    public Logger (){
        loggingString = new StringBuilder();
        new File("/sdcard/tmp/MotorLogs").mkdirs();
        new File("/sdcard/tmp/ServoLogs").mkdirs();
        new File("/sdcard/tmp/DualServoLogs").mkdirs();
        new File("/sdcard/tmp/CRServoLogs").mkdirs();
        new File("/sdcard/tmp/RobotLogs").mkdirs();
        try {
            logIndexReader = new Scanner(logIndex);
            data = logIndexReader.nextLine();
            logIndexer = new FileWriter(logIndex);

            char a = data.charAt(0);
            a++;
            if(a == '9'){
                a='0';
            }

            logIndexer.write(a);
            logIndexer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * Logs a Pose2d position ArrayList
     * @param p_position Pose2d parameter
     */

    @SuppressLint("SdCardPath")
    public void logPos(Pose2d p_position){
        try {
            FileWriter poser = new FileWriter("/sdcard/tmp/pos.csv");
            poser.write(p_position.getX()+"\n"+p_position.getY()+"\n"+p_position.getHeading());
            poser.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Returns the logged Pose2d position in the log file
     * @return updated Pose2d
     */

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

    /*  */

    /**
     * Creates a new file for logging given a file name and the headers for organization of information
     * @param p_fileName name of the file user wants to create
     * @param p_headers headers displayed at the top of the file log to organize data
     */

    @SuppressLint("SdCardPath")
    public void createFile (String p_fileName, String p_headers) {
        File file = new File("/sdcard/tmp/"+p_fileName+data+"Log.csv");
        headerList.computeIfAbsent(file, k -> new ArrayList<>());
        Objects.requireNonNull(headerList.get(file)).add(0);
        for (int i = 2; i < p_headers.length(); i++) {
            prevChar = p_headers.charAt(i - 1);
            if (prevChar == ' ' && p_headers.charAt(i) != prevChar) {
                Objects.requireNonNull(headerList.get(file)).add(i);
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

            fileWriter = new FileWriter(file, true);
            logList.put(p_fileName, file);
            currentTime = Calendar.getInstance().getTime().toString();

            fileWriter.write(currentTime + "\n");
            fileWriter.write(p_headers + "\n");
            for (int i = 0; i < p_headers.length(); i++) {
                fileWriter.write("=");
            }
            fileWriter.write("\n");
            fileWriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Logs inputted string given file name to write to
     * @param p_fileName name of the file user wants to create
     * @param p_input String parameter the user wants to log in the file
     */

    @SuppressLint("DefaultLocale")
    public void log(String p_fileName, String p_input) {
        try {
            @SuppressLint("SdCardPath")
            File file = new File("/sdcard/tmp/"+p_fileName+data+"Log.csv");
            FileWriter fileWriter = new FileWriter(file, true);
            fileWriter.write(String.format("%.2f", time) + ":" + p_input + "\n");
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Logs inputted string given file name to write to and has option to be formatted to headers
     * @param p_fileName name of the file user wants to create
     * @param p_input String parameter the user wants to log in the file
     * @param p_isFormatted Boolean parameter indicating whether user wants the data to be formatted to headers or not
     */

    @SuppressLint("DefaultLocale")
    public void log(String p_fileName, String p_input, boolean p_isFormatted){
        if (p_isFormatted) {
            loggingString.setLength(0);
            tempHeaderPositions = headerList.get(logList.get(p_fileName));
            prevInputStringStartIndex = 0;
            inputStrings.clear();

            for (int i = 1; i < p_input.length(); i++) {
                prevChar = p_input.charAt(i - 1);
                if (prevChar == ',') {
                    inputStrings.add(p_input.substring(prevInputStringStartIndex, i - 1));
                    prevInputStringStartIndex = i;
                }
            }

            inputStrings.add(p_input.substring(prevInputStringStartIndex));

            for (int i = 0; i < inputStrings.size(); i++) {

                while (true) {
                    assert tempHeaderPositions != null;
                    if (!(tempHeaderPositions.get(i + 1)- loggingString.length()-String.format("%.2f", time).length() - 1 > 0))
                        break;
                    loggingString.append(" ");
                }
                loggingString.append(inputStrings.get(i));
            }

            try {
                FileWriter fileWriter = new FileWriter(logList.get(p_fileName), true);
                fileWriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                fileWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        else {
            log(p_fileName, p_input);
        }
    }

    /**
     * Logs inputted string given file name to write to without the runtime at the beginning of the line
     * @param p_fileName name of the file user wants to create
     * @param p_input String parameter the user wants to log in the file
     */

    @SuppressLint("DefaultLocale")
    public void logNoTime(String p_fileName, String p_input){
        try {
            @SuppressLint("SdCardPath")
            File file = new File("/sdcard/tmp/"+p_fileName+data+"Log.csv");
            FileWriter fileWriter = new FileWriter(file, true);
            fileWriter.write(p_input + "\n");
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Logs inputted string given file name to write to and has option to be formatted to headers and the option
     * to be regulated to only log every 20 loop
     * @param p_fileName name of the file user wants to create
     * @param p_input String parameter the user wants to log in the file
     * @param p_isFormatted Boolean parameter indicating whether user wants the data to be formatted to headers or not
     * @param p_isRegulated Boolean parameter indicating whether user wants the data to be regulated from being logged
     * by time intervals
     */

    @SuppressLint("DefaultLocale")
    public void log(String p_fileName, String p_input, boolean p_isFormatted, boolean p_isRegulated){
        if (p_isFormatted && p_isRegulated) {
            loggingString.setLength(0);
            tempHeaderPositions = headerList.get(logList.get(p_fileName));
            prevInputStringStartIndex = 0;
            inputStrings.clear();

            if (loopCounter % 20 == 0) {

                for (int i = 1; i < p_input.length(); i++) {
                    prevChar = p_input.charAt(i - 1);
                    if (prevChar == ',') {
                        inputStrings.add(p_input.substring(prevInputStringStartIndex, i - 1));
                        prevInputStringStartIndex = i;
                    }
                }

                inputStrings.add(p_input.substring(prevInputStringStartIndex));

                for (int i = 0; i < inputStrings.size(); i++) {

                    while (true) {
                        assert tempHeaderPositions != null;
                        if (!(tempHeaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", time).length() - 1 > 0))
                            break;
                        loggingString.append(" ");
                    }
                    loggingString.append(inputStrings.get(i));
                }

                try {
                    FileWriter fileWriter = new FileWriter(logList.get(p_fileName), true);
                    fileWriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                    fileWriter.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }


        }

        else if (p_isFormatted) {
            log(p_fileName, p_input, true);
        }

        else if (p_isRegulated) {
            if (loopCounter % 20 == 0) {
                log(p_fileName, p_input);
            }
        }

        else {
            log(p_fileName, p_input);
        }
    }

    /**
     * Logs inputted string given file name to write to and has the option to be formatted to headers, the option
     * to be regulated to only log every 20 loops, and the option to also print the input as telemetry
     * @param p_fileName name of the file user wants to create
     * @param p_input String parameter the user wants to log in the file
     * @param p_isFormatted Boolean parameter indicating whether user wants the data to be formatted to headers or not
     * @param p_isRegulated Boolean parameter indicating whether user wants the data to be regulated from being logged
     * by time intervals
     * @param p_isTelemetry Boolean parameter indicating whether user wants the data to also be outputted as telemetry
     * or not
     */

    @SuppressLint("DefaultLocale")
    public void log(String p_fileName, String p_input, boolean p_isFormatted, boolean p_isRegulated, boolean p_isTelemetry){
        if (p_isFormatted && p_isRegulated) {
            loggingString.setLength(0);
            tempHeaderPositions = headerList.get(logList.get(p_fileName));
            prevInputStringStartIndex = 0;
            inputStrings.clear();

            if (loopCounter % 20 == 0) {

                for (int i = 1; i < p_input.length(); i++) {
                    prevChar = p_input.charAt(i - 1);
                    if (prevChar == ',') {
                        inputStrings.add(p_input.substring(prevInputStringStartIndex, i - 1));
                        prevInputStringStartIndex = i;
                    }
                }

                inputStrings.add(p_input.substring(prevInputStringStartIndex));

                for (int i = 0; i < inputStrings.size(); i++) {

                    while (true) {
                        assert tempHeaderPositions != null;
                        if (!(tempHeaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", time).length() - 1 > 0))
                            break;
                        loggingString.append(" ");
                    }
                    loggingString.append(inputStrings.get(i));
                }

                try {
                    FileWriter fileWriter = new FileWriter(logList.get(p_fileName), true);
                    fileWriter.write(String.format("%.2f", time) + ":" + loggingString + "\n");
                    fileWriter.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }


        }

        else if (p_isFormatted) {
            log(p_fileName, p_input, true);
        }

        else if (p_isRegulated) {
            if (loopCounter % 20 == 0) {
                log(p_fileName, p_input);
            }
        }

        else if (p_isTelemetry) {
            log(p_fileName, p_input);
            op.telemetry.addData("Log Telemetry: ", p_input);
        }

        else {
            log(p_fileName, p_input);
        }
    }

//    public void logMessage(String fileName, String input){
////        if (loopCounter % 5 == 0) {
//        try {
//            FileWriter fileWriter = new FileWriter(logList.get(fileName), true);
//            fileWriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
//            fileWriter.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
////        }
//    }

//    @SuppressLint("DefaultLocale")
//    public void logRegulated(String fileName, String input){
//
//        loggingString = "";
//        tempHeaderPositions = headerList.get(logList.get(fileName));
//        prevInputStringStartIndex = 0;
//        inputStrings.clear();
//
//        if (loopCounter % 20 == 0) {
//
//            for (int i = 1; i < input.length(); i++) {
//                prevChar = input.charAt(i - 1);
//                if (prevChar == ',') {
//                    inputStrings.add(input.substring(prevInputStringStartIndex, i - 1));
//                    prevInputStringStartIndex = i;
//                }
//            }
//
//            inputStrings.add(input.substring(prevInputStringStartIndex));
//
//            for (int i = 0; i < inputStrings.size(); i++) {
//
//                while (tempHeaderPositions.get(i + 1) - loggingString.length() - String.format("%.2f", op.getRuntime()).length() - 1 > 0) {
//                    loggingString += " ";
//                }
//                loggingString += inputStrings.get(i);
//            }
//
//            try {
//                FileWriter fileWriter = new FileWriter(logList.get(fileName), true);
//                fileWriter.write(String.format("%.2f", op.getRuntime()) + ":" + loggingString + "\n");
//                fileWriter.close();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//
//        }
//    }
//
//    public void logRegulatedMessage(String fileName, String input) {
//        if (loopCounter % 10 == 0) {
//            try {
//                FileWriter fileWriter = new FileWriter(logList.get(fileName), true);
//                fileWriter.write(String.format("%.2f", op.getRuntime()) + ":" + input + "\n");
//                fileWriter.close();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }

    /**
     * Closes log files
     */

    public void closeLog(){
        File[] fileArray = logList.values().toArray(new File[1]);
        try {
            for (File value : fileArray) {
                FileWriter file = new FileWriter(value);
                file.close();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }
}
