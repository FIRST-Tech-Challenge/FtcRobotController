package org.firstinspires.ftc.teamcode.rework;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;

public class TelemetryDump {
    Telemetry telemetry;
    LinkedHashMap<String,String> opModeData;
    LinkedHashMap<String,String> executorModuleData;

    public TelemetryDump(Telemetry telemetry){
        this.telemetry = telemetry;
        opModeData = new LinkedHashMap<>();
        executorModuleData = new LinkedHashMap<>();
    }

    public void addData(String s, String val){
        if(Thread.currentThread().getName().equals("module executor")){
            executorModuleData.put(s,val);
        }else {
            opModeData.put(s, val);
        }
    }

    public void addHeader(String s){
        addData(s,"");
    }

    public void addData(String s, double val){
        if(Thread.currentThread().getName().equals("module executor")){
            executorModuleData.put(s,Double.toString(val));
        }else {
            opModeData.put(s,Double.toString(val));
        }
    }

    public void update(){
        StringBuilder out = new StringBuilder();
        for(String key : executorModuleData.keySet()){
            out.append(key).append(executorModuleData.get(key)).append("\n");
        }
        for(String key : opModeData.keySet()){
            out.append(key).append(opModeData.get(key)).append("\n");
        }
        telemetry.addLine(out.toString());
        telemetry.update();
    }
}
