package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.util.RobotLog;

/** Each independent mechanism gets one stop attempt even if another throws. */
public final class SafeShutdown {
    private SafeShutdown() { }
    public static void run(Runnable... stops) {
        for(Runnable stop:stops)try{stop.run();}catch(RuntimeException e){RobotLog.ee("BioBuzzStop",e,"Hardware stop failed; SDK failsafe remains enabled");}
    }
}
