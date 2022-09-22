package org.firstinspires.ftc.teamcode.ebotsutil;

import android.util.Log;

import static java.lang.String.format;

public class StopWatch {
    private long startTime;

    //This constructor was refactored 11/11 to start the timer during instantiation
    public StopWatch(){
        startTime = System.currentTimeMillis();
    }

    public void startTimer(){
        startTime = System.currentTimeMillis();  //current time in milliseconds
    }

    public void reset(){
        this.startTimer();
    }

    public long getElapsedTimeMillis(){
        return startTime == 0 ? 0L : System.currentTimeMillis() - startTime;
    }

    public double getElapsedTimeSeconds(){
        return startTime == 0 ? 0.0 : ((System.currentTimeMillis() - startTime) / 1000.0);
    }

    public long logSplitTime(String logTag, String operation, long startTimeMillis, int loopCount){
        long newTime = this.getElapsedTimeMillis();
        long duration = newTime - startTimeMillis;
        float avgLoopTime = (float) newTime / loopCount;
        float percentageOfAvgLoop = 100.0f * duration / avgLoopTime;
        Log.d(logTag, "Loop #" + loopCount + ": "+ operation + " took " + duration + " ms which is "
                + String.format("%.1f", percentageOfAvgLoop) + "% of " + String.format("%.0f", avgLoopTime)
                + " ms average loop time");
        return newTime;
    }

    @Override
    public String toString(){
        return String.format("%.2f", getElapsedTimeSeconds()) + " seconds";
    }

    public String toString(int loopCount){
        Double frequency = loopCount / getElapsedTimeSeconds();
        return loopCount + " loops in " + getElapsedTimeMillis() + " ms or " + String.format("%.2f", frequency) + " Hz";
    }

    public String toString(int loopCount, long loopDuration){
        Double frequency = loopCount / getElapsedTimeSeconds();
        return loopDuration + " ms current loop (" + String.format("%.1f", (float) 1000/loopDuration) + " Hz).  Cummulative stats: " + loopCount
                + " loops in " + getElapsedTimeMillis() + " ms or "
                + String.format("%.2f", frequency) + " Hz";
    }

}

