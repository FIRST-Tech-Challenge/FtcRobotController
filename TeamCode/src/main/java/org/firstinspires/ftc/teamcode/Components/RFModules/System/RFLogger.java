package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import android.annotation.SuppressLint;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.LogRecord;
import java.util.logging.SimpleFormatter;

public class RFLogger {
    public static Logger LOGGER;
    ArrayList<FileHandler> handlerList = new ArrayList<>();
    StackTraceElement[] elements = new StackTraceElement[0];
    SimpleFormatter sh = new SimpleFormatter();
    FileHandler TestFH;
    Level logLevel = Level.INFO;
    static FileHandler GeneralFH, AutonomousFH, HardwareFH, QueuerFH;

    public enum Files {
        GENERAL_LOG("/sdcard/tmp/Generalowo.log", 0),
        AUTONOMOUS_LOG("/sdcard/tmp/Auton.log", 1),
        HARDWARE_LOG("/sdcard/tmp/Hardware.log", 2),
        QUEUER_LOG("/sdcard/tmp/Queuer.log", 3);

        String filePath;
        int index;

        Files(String p_filePath, int p_index){
            filePath = p_filePath;
            index = p_index;
        }
    }

    public enum Severity {
        ALL(Level.ALL),
        SEVERE(Level.SEVERE),
        WARNING(Level.WARNING),
        INFO(Level.INFO),
        CONFIG(Level.CONFIG),
        FINE(Level.FINE),
        FINER(Level.FINER),
        FINEST(Level.FINEST);

        Level logSeverity;

        Severity(Level p_logSeverity){
            logSeverity = p_logSeverity;
        }
    }

    @SuppressLint("SdCardPath")
    public RFLogger (String className){
        LOGGER = Logger.getLogger(className);
        LOGGER.setLevel(logLevel);
        try {
            GeneralFH = new FileHandler(Files.GENERAL_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            AutonomousFH = new FileHandler(Files.AUTONOMOUS_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            HardwareFH = new FileHandler(Files.HARDWARE_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            QueuerFH = new FileHandler(Files.QUEUER_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            TestFH = new FileHandler("/sdcard/tmp/Testinglog.log");
        } catch (IOException e) {
            e.printStackTrace();
        }

        handlerList.add(GeneralFH);
        handlerList.add(AutonomousFH);
        handlerList.add(HardwareFH);
        handlerList.add(QueuerFH);

        TestFH.setFormatter(new SimpleFormatter() {
            private static final String format = "[%1$tF %1$tT.%1$tL] [%2$-7s] %3$s %n";
            @SuppressLint("DefaultLocale")
            public synchronized String format(LogRecord lr) {
                return String.format(format,
                        new Date(lr.getMillis()),
                        lr.getLevel().getLocalizedName(),
                        lr.getMessage()
                );
            }
        });
        AutonomousFH.setFormatter(sh);
        HardwareFH.setFormatter(sh);
        QueuerFH.setFormatter(sh);

        LOGGER.addHandler(GeneralFH);

    }

    public void setLogLevel(Severity p_severity){
        logLevel = p_severity.logSeverity;
        LOGGER.setLevel(logLevel);
    }

    //    public void log(Severity p_severity, Files p_file, String info){
//        String output = "";
//        fh = handlerList.get(p_file.index);
//        LOGGER.addHandler(fh);
//        logLevel = p_severity.logSeverity;
//        elements = Thread.currentThread().getStackTrace();
//        for(int i = 0;i<elements.length; i++){
//            output += ".";
//        }
//        LOGGER.log(logLevel, output);
//
//    }
    public void log(Severity p_severity, String info){
        StringBuilder output = new StringBuilder(":");
        StringBuilder maxMethods = new StringBuilder("");
        logLevel = p_severity.logSeverity;
        StackTraceElement[] elements = Thread.currentThread().getStackTrace();
        boolean first = false;
        StackTraceElement firstElement = elements[0];
        for (StackTraceElement element : elements) {
            maxMethods.append("\n" + "   " + element.getFileName() + ": " + element.getClassName() + "." + element.getMethodName());
            if (element.toString().startsWith("org")) {
                output.append(".");
                if(!first && !element.getMethodName().startsWith("log")){
                    first = true;
                    firstElement = element;
                }
            }
        }
        LOGGER.log(logLevel, firstElement.getMethodName() + output + info + maxMethods);
    }

    public void log(FileHandler p_file, String info){
        StringBuilder output = new StringBuilder(":");
//        fh = handlerList.get(p_file.index);
//        LOGGER.addHandler(fh);
        LOGGER.addHandler(p_file);
        elements = Thread.currentThread().getStackTrace();
        for (StackTraceElement element : elements) {
            if (element.getClassName().startsWith("TeamCode")) {
                output.append(".");
            }
        }
        LOGGER.log(logLevel, output + info);
    }

    public void log(String info){
        StringBuilder output = new StringBuilder(":");
//            fh = handlerList.get(0);
//            LOGGER.addHandler(fh);
//        LOGGER.addHandler(TestFH);
        StackTraceElement[] elements = Thread.currentThread().getStackTrace();
        boolean first = false;
        StackTraceElement firstElement = elements[0];
        for (StackTraceElement element : elements) {
            if (element.toString().startsWith("org")) {
                output.append(".");
                if(!first && !element.getMethodName().startsWith("log")){
                    first = true;
                    firstElement = element;
                }
            }
        }
        LOGGER.log(logLevel, firstElement.getMethodName() + output + info);
    }
    public void logMAX(String info){
        StringBuilder output = new StringBuilder(":");
        StringBuilder maxMethods = new StringBuilder("");
        StackTraceElement[] elements = Thread.currentThread().getStackTrace();
        boolean first = false;
        StackTraceElement firstElement = elements[0];
        for (StackTraceElement element : elements) {
            maxMethods.append("\n" + "   " + element.getFileName() + ": " + element.getClassName() + "." + element.getMethodName());
            if (element.toString().startsWith("org")) {
                output.append(".");
                if(!first && !element.getMethodName().startsWith("log")){
                    first = true;
                    firstElement = element;
                }
            }
        }
        LOGGER.log(logLevel, firstElement.getMethodName() + output + info + maxMethods);
    }
}
