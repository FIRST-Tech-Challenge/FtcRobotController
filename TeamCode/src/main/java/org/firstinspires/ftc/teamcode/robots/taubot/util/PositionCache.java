package org.firstinspires.ftc.teamcode.robots.taubot.util;
import android.content.SharedPreferences;

import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.RC;


public class PositionCache {
    private int updateInterval;
    SharedPreferences sharedPref = RC.a().getPreferences(RC.c().MODE_PRIVATE);
    SharedPreferences.Editor editor = sharedPref.edit();
    Gson gson = new Gson();
    private int cyclesSinceUpdate = 0;

    public PositionCache(int updateInterval){
        this.updateInterval = updateInterval;
    }

    public void writePose(TauPosition pos, boolean forceFlush) {
        pos.updateTime();
        String json = gson.toJson(pos);
        editor.putString("TauPosition", json);
        if (forceFlush)
            editor.commit();
        else
            editor.apply();
    }

    public TauPosition readPose () {
        String json = sharedPref.getString("TauPosition", "get failed"); //retrieves the shared preference
        if(json.equals("get failed")) return new TauPosition(); //return a default zeroed TauPos if there's nothing in shared preferences
        return gson.fromJson(json, TauPosition.class); //load the saved JSON into the cached class
    }

    public int update(TauPosition pos, boolean forceUpdate){
        if(!forceUpdate){
            if(cyclesSinceUpdate%updateInterval==0){
                writePose(pos, false);
            }
        }
        else{
            writePose(pos, true);
        }
        cyclesSinceUpdate++;
        return cyclesSinceUpdate;
    }


}

