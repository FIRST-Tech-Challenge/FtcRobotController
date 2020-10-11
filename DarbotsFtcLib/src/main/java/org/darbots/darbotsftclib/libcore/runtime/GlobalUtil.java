package org.darbots.darbotsftclib.libcore.runtime;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.Number_Log;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.lang.reflect.Field;

public class GlobalUtil {
    private static Field lynxModuleLastDataField = null;
    public static LogLevel LowestLogLevel = LogLevel.INFO;
    public static void addLog(String module, String caption, LogContent content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = content;
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }
    public static void addLog(String module, String caption, String content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = new String_Log(content);
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }
    public static void addLog(String module, String caption, Number content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = new Number_Log(content);
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }

    public static void debugOutputLog(String module, String caption, LogContent logContent, LogLevel logLevel){
        System.out.print("[" + logLevel.name() + "]");
        System.out.print(module + "::" + caption);
        System.out.println(" | " + logContent.getContentValue().toString());
        RobotLog.dd(module,caption + ":" + logContent.getContentValue().toString());
    }


    public static void deleteAllLogs(){
        File[] logFiles = FTCFileIO.getLogFolder().listFiles();
        for(int i=0;i<logFiles.length;i++){
            if(logFiles[i].isFile())
                logFiles[i].delete();
        }
    }

    public static Telemetry getTelemetry(){
        if(GlobalRegister.runningOpMode != null){
            return GlobalRegister.runningOpMode.telemetry;
        }else{
            return null;
        }
    }

    public static RobotGyro getGyro(){
        if(GlobalRegister.runningOpMode != null){
            if(GlobalRegister.runningOpMode.getRobotCore() != null){
                return GlobalRegister.runningOpMode.getRobotCore().getGyro();
            }
        }
        return null;
    }
    public static void OpModeInitialized(){
        return;
    }
    public static void OpModeStarted(){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.startOpMode();
        }
    }

    public static void OpModeEnded(){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.endOpMode();
        }
    }

    public static void updateBulkRead(){
        if(GlobalRegister.allExtensionHubs == null){
            return;
        }
        if(lynxModuleLastDataField == null){
            setupLynxModuleReflection();
        }
        for(LynxModule i : GlobalRegister.allExtensionHubs){
            i.clearBulkCache();
            try {
                lynxModuleLastDataField.set(i,i.getBulkData());
            }catch(IllegalAccessException e){
                e.printStackTrace();
            }
        }
    }

    public static void updateGlobalGyro(){
        if(GlobalRegister.currentRobotCore == null){
            return;
        }
        RobotGyro gyro = GlobalRegister.currentRobotCore.getGyro();
        if(gyro != null && gyro instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) gyro).updateStatus();
        }
    }

    public static void updateGyro(RobotGyro gyro){
        if(gyro != null && gyro instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) gyro).updateStatus();
        }
    }

    public static void setDataUpdateMethod(LynxModule.BulkCachingMode Mode){
        if(GlobalRegister.allExtensionHubs == null){
            return;
        }
        for(LynxModule i : GlobalRegister.allExtensionHubs){
            i.setBulkCachingMode(Mode);
        }
        if(Mode != LynxModule.BulkCachingMode.OFF) {
            setupLynxModuleReflection();
            updateBulkRead();
        }
    }

    private static void setupLynxModuleReflection(){
        try {
            lynxModuleLastDataField = LynxModule.class.getDeclaredField("lastBulkData");
            lynxModuleLastDataField.setAccessible(true);
        }catch(NoSuchFieldException e){
            e.printStackTrace();
        }
    }

    public static void addTelmetryLine(Telemetry telemetry, TelemetryPacket telemetryPacket, String title, String value){
        if(telemetry != null){
            telemetry.addData(title,value);
        }
        if(telemetryPacket != null){
            telemetryPacket.addLine(title + ": " + value);
        }
    }
}
