package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import androidx.annotation.Nullable;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

public class DataLogger {
    @SuppressLint("SdCardPath")
    private static final String DIRECTORY_PATH = "/sdcard/FIRST/SympleLogs";

    @Nullable
    private FileWriter fileWriter;

    public DataLogger(String filePrefix, boolean suppress) {
        String fileName =  createFileName(filePrefix);
        String filePath = DIRECTORY_PATH + "/" + fileName;

        new File(DIRECTORY_PATH).mkdir(); // create the directory if not exists

        try {
            if(!suppress) {
                this.fileWriter = new FileWriter(filePath, true);
                this.writeLine("// " + fileName);
            }
        } catch (IOException ignored) { }
    }

    public DataLogger(String filePrefix) {
        this(filePrefix, false);
    }

    private String createFileName(String prefix) {
        return prefix + "_" + getCurrentTime("dd-MM-yyyy@HH-mm-ss") + ".txt";
    }

    public void addData(DataType dataType, String data) {
        String linePrefix = "<" + getCurrentTime("dd-MM-yyyy | HH:mm:ss.SSS") + " / " + dataType.name() + "> ";
        try {
            this.writeLine(linePrefix + data);
        } catch (IOException ignored) { }
    }

    public void addData(DataType dataType, boolean bool) {
        addData(dataType, Boolean.toString(bool));
    }

    public void addData(DataType dataType, int i) {
        addData(dataType, Integer.toString(i));
    }

    public void addData(DataType dataType, long l) {
        addData(dataType, Long.toString(l));
    }

    public void addData(DataType dataType, double d) {
        addData(dataType, Double.toString(d));
    }

    public void addData(DataType dataType, float f) {
        addData(dataType, Float.toString(f));
    }

    public void addData(DataType dataType, Object o) {
        addData(dataType, String.valueOf(o));
    }

    public void addThrowable(Throwable throwable) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        throwable.printStackTrace(pw);
        String sStackTrace = sw.toString();
        addData(DataType.ERROR, sStackTrace);
    }

    public void closeFile() {
        if(fileWriter == null) return;
        try {
            fileWriter.close();
        } catch (IOException ignored) { }
    }

    private void writeLine(String data) throws IOException {
        if(fileWriter == null) return;
        fileWriter.write(data + System.lineSeparator());
        flushData();
    }

    private void flushData() throws IOException {
        if(fileWriter == null) return;
        fileWriter.flush();
    }

    private String getCurrentTime(String format) {
        @SuppressLint("SimpleDateFormat")
        SimpleDateFormat formatter = new SimpleDateFormat(format);

        formatter.setTimeZone(TimeZone.getTimeZone("Asia/Jerusalem"));

        return formatter.format(new Date());
    }

    @Override
    protected void finalize() throws Throwable {
        closeFile();
        flushData();
        super.finalize();
    }

    public enum DataType {
        INFO,
        WARN,
        ERROR
    }
}
