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
import java.util.logging.Filter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.LogRecord;
import java.util.logging.SimpleFormatter;

// default verbosity = INFO
// maximum verbosity = FINEST
// global verbosity level
// print name of program at top of logfile
// in competition opmodes = only info verbosity

public class RFLogger {
    public Logger LOGGER;
    ArrayList<FileHandler> handlerList = new ArrayList<>();
    Level logLevel = Level.ALL;
    static FileHandler GeneralFH, AutonomousFH, HardwareFH, QueuerFH;
    public static final Severity FILTER = Severity.INFO;


    public enum Files {
        GENERAL_LOG("/sdcard/tmp/General.log",0),
        AUTONOMOUS_LOG("/sdcard/tmp/Autonomous.log", 1),
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
        ALL(Level.SEVERE),
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

        handlerList.add(GeneralFH);
        handlerList.add(AutonomousFH);
        handlerList.add(HardwareFH);
        handlerList.add(QueuerFH);

        SimpleFormatter customSH = new SimpleFormatter() {
            private static final String format = "[%1$tF %1$tT.%1$tL] [%2$-7s] %3$s %n";
            @SuppressLint("DefaultLocale")
            public synchronized String format(LogRecord lr) {
                return String.format(format,
                        new Date(lr.getMillis()),
                        lr.getLevel().getLocalizedName(),
                        lr.getMessage()
                );
            }
        };

        GeneralFH.setFormatter(customSH);
        AutonomousFH.setFormatter(customSH);
        HardwareFH.setFormatter(customSH);
        QueuerFH.setFormatter(customSH);

        LOGGER.addHandler(GeneralFH);
    }

    public void setLogLevel(Severity p_severity){
        logLevel = p_severity.logSeverity;
        LOGGER.setLevel(logLevel);
    }

    public void log(String info){
        log(Files.GENERAL_LOG, info);
    }

    public void log(Severity p_severity, String info){
        setLogLevel(p_severity);
        log(info);
    }

    public void log(Files p_file, String info){
        if(logLevel.intValue()>=FILTER.logSeverity.intValue()) {
            for (Handler i : LOGGER.getHandlers()) {
                LOGGER.removeHandler(i);
            }
            LOGGER.addHandler(handlerList.get(p_file.index));
            StringBuilder output = new StringBuilder(":");
            StackTraceElement[] elements = Thread.currentThread().getStackTrace();
            boolean first = false;
            StackTraceElement firstElement = elements[0];
            for (StackTraceElement element : elements) {
                if (element.toString().startsWith("org")) {
                    output.append(".");
                    if (!first && !element.getMethodName().startsWith("log")) {
                        first = true;
                        firstElement = element;
                    }
                }
            }
            LOGGER.log(logLevel, firstElement.getMethodName() + output + info);
        }
    }

    public void log(Files p_file, Severity p_Severity, String info){
        setLogLevel(p_Severity);
        log(p_file, info);
    }

//    public void logMAX(String info){
//        StringBuilder output = new StringBuilder(":");
//        StringBuilder maxMethods = new StringBuilder();
//        StackTraceElement[] elements = Thread.currentThread().getStackTrace();
//        boolean first = false;
//        StackTraceElement firstElement = elements[0];
//        for (StackTraceElement element : elements) {
//            maxMethods.append("\n" + "   " + element.getFileName() + ": " + element.getClassName() + "." + element.getMethodName());
//            if (element.toString().startsWith("org")) {
//                output.append(".");
//                if(!first && !element.getMethodName().startsWith("log")){
//                    first = true;
//                    firstElement = element;
//                }
//            }
//        }
//        LOGGER.log(logLevel, firstElement.getMethodName() + output + info + maxMethods);
//    }
}
