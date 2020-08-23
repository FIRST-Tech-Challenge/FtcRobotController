package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.HashMap;

public class FileDump {
    HashMap<String,StringBuilder> files;
    long startTime;

    public FileDump(){
        startTime = SystemClock.elapsedRealtime();
        files = new HashMap<>();
    }

    public synchronized void addData(String fileName, String data){
        if(files.containsKey(fileName)) {
            files.put(fileName, files.get(fileName).append(data).append("\n"));
        }else{
            files.put(fileName,new StringBuilder(data).append("\n"));
        }
    }

    public synchronized void writeFilesToDevice(){
        for(String key : files.keySet()){
            files.get(key).insert(0,"x y\n");
            writeToFile(Long.toString(startTime),key,files.get(key).toString());
        }
    }

    public void writeToFile(String directoryName, String fileName, String data) {
        File captureDirectory = new File(AppUtil.ROBOT_DATA_DIR, "/" + directoryName + "/");
        if (!captureDirectory.exists()) {
            boolean isFileCreated = captureDirectory.mkdirs();
            Log.d("DumpToFile", " " + isFileCreated);
        }
        Log.d("DumpToFile", " hey ");
        File file = new File(captureDirectory, fileName);
        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            OutputStreamWriter writer = new OutputStreamWriter(outputStream);
            try {
                writer.write(data);
                writer.flush();
                Log.d("DumpToFile", data);
            } finally {
                outputStream.close();
                Log.d("DumpToFile", file.getAbsolutePath());
            }
        } catch (IOException e) {
            RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
        }
    }
}
