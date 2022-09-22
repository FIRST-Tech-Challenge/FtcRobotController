package org.firstinspires.ftc.teamcode.ebotsenums;

import android.util.Log;

import java.util.Formatter;

public enum SoftStart {
    NO (false, 0L, 0.2),
    MEDIUM (true, 750L, 0.2),
    SLOW_START(true, 1200L, 0.2);

    /**  ENUM VARIABLES     **************/
    private boolean softStartOn;
    private long durationMillis;
    private double minPower;

    /**  CONSTRUCTOR    **************/
    SoftStart(boolean softStartOn, long durationMillis, double minPower){
        this.softStartOn = softStartOn;
        this.durationMillis = durationMillis;
        this.minPower = minPower;

    }
    /**  ENUM GETTERS AND SETTERS  ***********/
    public boolean isSoftStartOn(){return this.softStartOn;}
    public long getDurationMillis(){return  this.durationMillis;}
    public double getMinPower(){return minPower;}

    /**  ENUM Functions  ***********/
    public double getScaleFactor(long currentTimeMillis){
        //todo:  Consider if need to utilize minPower setting
        //       Currently, this ignores minPower and softStartOn
        String logTag = "EBOTS";
        boolean debugOn = false;
        if (debugOn) Log.d(logTag, "Entering SoftStart.getScaleFactor...");

        double scaleFactor = 1.0;
        if(currentTimeMillis < durationMillis){
            scaleFactor = ((double)currentTimeMillis/ (double) durationMillis);
            if (scaleFactor<minPower) scaleFactor=minPower;
        }

        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("currentTimeMillis: ");
            sb.append(currentTimeMillis);
            sb.append(", durationMillis: ");
            sb.append(durationMillis);
            sb.append(", scaleFactor:");
            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f", scaleFactor);
            Log.d(logTag, sb.toString());
        }

        return scaleFactor;
    }
}
