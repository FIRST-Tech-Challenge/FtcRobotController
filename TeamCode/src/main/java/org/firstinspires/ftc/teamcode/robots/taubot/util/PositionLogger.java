package org.firstinspires.ftc.teamcode.robots.taubot.util;
import android.content.SharedPreferences;
import android.os.Environment;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.time.LocalTime;
import java.util.Scanner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.RC;


public class PositionLogger{
    private int updateInterval;
    SharedPreferences sharedPref = RC.a().getPreferences(RC.c().MODE_PRIVATE);
    SharedPreferences.Editor editor = sharedPref.edit();
    Gson gson = new Gson();
    private int cyclesSinceUpdate = 0;

    public PositionLogger(int updateInterval){
        this.updateInterval = updateInterval;
    }

    public void writePose(TauPosition pos) {
        pos.updateTime();
        String json = gson.toJson(pos);
        editor.putString("TauPosition", json);
        editor.apply();
    }

    public TauPosition readPose () {
        String json = sharedPref.getString("TauPosition", "get failed");
        if(json.equals("get failed")) return new TauPosition();
        return gson.fromJson(json, TauPosition.class);
    }

    public int update(TauPosition log, boolean forceUpdate){
        if(!forceUpdate){
            if(cyclesSinceUpdate%updateInterval==0){
                writePose(log);
            }
        }
        else{
            writePose(log);
        }
        cyclesSinceUpdate++;
        return cyclesSinceUpdate;
    }

    public void closeLog(){
        try{
            //out.close();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
    }


}

