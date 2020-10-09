package org.firstinspires.ftc.teamcode.ultimategoal.util;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.HashMap;

public class FileDump {
    private ArrayList<FileDumpProvider> providers;
    HashMap<String, FileData> files;
    long startTime;

    public FileDump() {
        startTime = SystemClock.elapsedRealtime();
        files = new HashMap<>();
    }

    public void doTick() {
        providers.forEach(provider -> {
            if (files.containsKey(provider.getFileName())) {
                files.get(provider.getFileName()).text.append(provider.getFileData()).append("\n");
            } else {
                files.put(provider.getFileName(), new FileData("", new StringBuilder(provider.getFileData()).append("\n")));
            }
        });
    }

    public void writeFilesToDevice() {
        for (String key : files.keySet()) {
            writeToFile(Long.toString(startTime), key, files.get(key).toString());
        }
    }

    private void writeToFile(String directoryName, String fileName, String data) {
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

class FileData {
    public StringBuilder text;
    public String header;

    public FileData(String header, StringBuilder text) {
        this.header = header;
        this.text = text;
    }

    public String toString() {
        return text.insert(0, header).toString();
    }
}
