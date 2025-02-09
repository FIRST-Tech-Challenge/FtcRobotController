// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class BotBot {

    public HardwareMap hwMap = null;
    protected Telemetry telemetry;
    public LinearOpMode opMode;

    OutputStreamWriter onLoopWriter;

    public static final int SIDE_RED = 0;
    public static final int SIDE_BLUE = 1;

    public boolean isAuto = true;

    public BotBot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        try {
            onLoopWriter = new FileWriter("/sdcard/FIRST/onlooplog_" + java.text.DateFormat.getDateTimeInstance().format(new Date()) + ".csv", true);
        } catch (IOException e) {
            throw new RuntimeException("onloop file writer open failed: " + e.toString());
        }
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
    }

    public void onLoop(String label){
        onLoop(100, label);
    }
    long lastOnLoopFinished = 0;
    String lastOnLoopLabel = "";
    int onLoopTolerance = 400;
    public void onLoop(int interval, String label){
        long start = System.currentTimeMillis();
        // TRICKY : DEBUG feature, please comment following block out before competition
//        if (lastOnLoopFinished > 0 && start - lastOnLoopFinished > (interval + onLoopTolerance)){
//            close();
//            throw new RuntimeException("onLoop(" + label + ") has been called too long (" + (start - lastOnLoopFinished) + ") ago, last onLoop label is "+lastOnLoopLabel);
//        }
        //RobotLog.d("FourWDBot OnLoop start ");
        this.onTick();
        long timeElapsed = System.currentTimeMillis() - start;
        //opMode.telemetry.addData("loop:", timeElapsed);
        RobotLog.d("FourWDBot OnLoop stop @ " + timeElapsed);
        // TRICKY : DEBUG feature, please comment following block out before competition
//        if (timeElapsed > interval){
//            close();
//            throw new RuntimeException("onTick(" + label + ") took too long (" + timeElapsed + ") to finish, last onLoop label is " + lastOnLoopLabel);
//        }
        try {
            RobotLog.d("onLoopWriter.write");
            RobotLog.d(String.format("%d, %d, %d, %s\n", interval, timeElapsed, start - lastOnLoopFinished, label));
            onLoopWriter.write(String.format("%d, %d, %d, %s\n", interval, timeElapsed, start - lastOnLoopFinished, label));
        } catch (IOException e) {
            throw new RuntimeException("onloop file writer write failed: " + e.toString());
        }
        if (interval > timeElapsed) {
            opMode.sleep(interval - (int) timeElapsed);
        }
        lastOnLoopFinished = System.currentTimeMillis();
        lastOnLoopLabel = label;
    }

    protected void onTick(){

    }

    // define event type sample rolled in
    public static final int EVENT_SAMPLE_ROLLED_IN = 1;
    public static final int EVENT_SAMPLE_ROLLED_OUT = 2;
    public static final int EVENT_PRELOAD_POSITION_ARRIVED = 1;

    public static final int EVENT_PRELOAD_SCORED = 2;

    public static final int EVENT_SAMPLE_1_PICKEDUP = 3;
    public static final int EVENT_SAMPLE_1_DROPPEDOFF= 4;
    public static final int EVENT_SAMPLE_2_PICKEDUP = 5;
    public static final int EVENT_SAMPLE_2_DROPPEDOFF = 6;
    public static final int EVENT_SAMPLE_3_PICKEDUP = 7;
    public static final int EVENT_SAMPLE_3_DROPPEDOFF = 8;
    public static final int EVENT_SPECIMEN_1_LOADED = 9;
    public static final int EVENT_SPECIMEN_1_SCORED = 10;
    public static final int EVENT_SPECIMEN_2_LOADED = 11;
    public static final int EVENT_SPECIMEN_2_SCORED = 12;
    public static final int EVENT_SPECIMEN_3_LOADED = 13;
    public static final int EVENT_SPECIMEN_3_SCORED = 14;
    public static final int EVENT_PARKED = 15;


    public static final String[] EVENT_NAMES = {
            "EVENT_SAMPLE_ROLLED_IN",
            "EVENT_SAMPLE_ROLLED_OUT",
            "EVENT_PRELOAD_POSITION_ARRIVED",
            "EVENT_PRELOAD_SCORED",
            "EVENT_SAMPLE_1_PICKEDUP",
            "EVENT_SAMPLE_1_DROPPEDOFF",
            "EVENT_SAMPLE_2_PICKEDUP",
            "EVENT_SAMPLE_2_DROPPEDOFF",
            "EVENT_SAMPLE_3_PICKEDUP",
            "EVENT_SAMPLE_3_DROPPEDOFF",
            "EVENT_SPECIMEN_1_LOADED",
            "EVENT_SPECIMEN_1_SCORED",
            "EVENT_SPECIMEN_2_LOADED",
            "EVENT_SPECIMEN_2_SCORED",
            "EVENT_SPECIMEN_3_LOADED",
            "EVENT_SPECIMEN_3_SCORED",
            "EVENT_PARKED"
    };

    public static final int EVENT_SAMPLE_1_SCORED = 4;
    public static final int EVENT_SAMPLE_2_SCORED = 6;
    public static final int EVENT_SAMPLE_3_SCORED = 8;





    protected void triggerEvent(int type, int data){
        onEvent(type, data);
    }
    protected void triggerEvent(int type){
        triggerEvent(type, 0);
    }

    protected void onEvent(int type, int data){

    }



    public void close(){
        shutdownTimer();
        try {
            RobotLog.d("onLoopWriter.close");
            onLoopWriter.close();
        } catch (IOException e) {
            throw new RuntimeException("onloop file writer close failed: " + e.toString());
        }
    }

    /**
     * Not blocking sleep, sleep n milliseconds while keep the onLoop() called every 100 milliseconds
     * @param milliseconds
     * @param label
     */
    public void sleep(int milliseconds, String label){
        ElapsedTime sleepTimer = new ElapsedTime();
        onLoop(10, label);
        while (sleepTimer.milliseconds() < milliseconds){
            onLoop(10, label);
        }
    }

    public void sleep(int milliseconds){
        sleep(milliseconds, "default sleep");
    }

    private final ScheduledExecutorService timer = Executors.newScheduledThreadPool(5);

    public void schedule(Runnable task, long delay){
        timer.schedule(task, delay, TimeUnit.MILLISECONDS);
    }
    public void shutdownTimer(){
        timer.shutdown();
    }


}

