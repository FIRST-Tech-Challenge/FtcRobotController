package org.firstinspires.ftc.teamcode.util;

/**
 * Created by Maximillian Virani on 3/19/2016.
 */

import android.os.Environment;

import com.qualcomm.robotcore.hardware.configuration.Utility;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class CsvLogKeeper {

    private static String variableName;
    private static FileWriter logKeeper;
    private static String fileName;
    private static boolean alreadyExists;
    private static File logFile;
    private static int dataLength;
    private Utility util;
    private String filesdir = Environment.getExternalStorageDirectory() + "/FIRST/";

    public CsvLogKeeper(String varname){

        dataLength = 1;
        variableName = varname;

        fileName = filesdir+varname+".csv";
        alreadyExists = new File(fileName).exists();
        logFile = new File(fileName);
        try{
            logKeeper = new FileWriter(logFile, true);
            if (!alreadyExists){
                logKeeper.append("Log of variable "+variableName);
                logKeeper.append("\n");
                logKeeper.flush();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public CsvLogKeeper(String title, int numvars, String varnames){

       dataLength = numvars;
        variableName = title;
        fileName = filesdir+title+".csv";
        alreadyExists = new File(fileName).exists();
        logFile = new File(fileName);
        try{
            logKeeper = new FileWriter(logFile, true);
            if (!alreadyExists){
                logKeeper.append("Log of variables "+variableName);
                logKeeper.append("\n");
                logKeeper.append(varnames);
                logKeeper.append("\n");
                logKeeper.flush();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(ArrayList data){
        try{
            for(int i=0; i<dataLength; i++){
                logKeeper.append(data.get(i).toString()+",");
            }
            logKeeper.append("\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(double data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(int data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(short data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(String data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(long data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(float data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(char data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(boolean data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(byte data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void UpdateLog(Object data){
        try{
            logKeeper.append(data.toString()+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void CloseLog(){
        try{
            logKeeper.append("\n");
            logKeeper.flush();
            logKeeper.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

}
