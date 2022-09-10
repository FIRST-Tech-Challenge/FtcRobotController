package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collection;
import java.util.HashMap;
import java.util.Scanner;

public class Logger {
    File myFile = new File("/storage/emulated/0/tmp/LogIndex.csv");
    Scanner myReader;


    HashMap<String, File> logList = new HashMap<>();

    FileWriter indexer;
    FileWriter filewriter = null;
    String  currentTime;
    public int loopcounter=0;
    String data = "0";
    public Logger (){

        try {
            myReader = new Scanner(myFile);
            data = myReader.nextLine();
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


    @SuppressLint("DefaultLocale")
    public void log(String fileName, String input){
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
    public void logRegulated(String fileName, String input){
        if (loopcounter % 10 == 0) {
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
