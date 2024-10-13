// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Date;

public class BotBot {

    public HardwareMap hwMap = null;
    public LinearOpMode opMode;

    OutputStreamWriter onLoopWriter;

    public static final int SIDE_RED = 0;
    public static final int SIDE_BLUE = 1;

    public boolean isAuto = true;

    public BotBot(LinearOpMode opMode) {
        this.opMode = opMode;
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

    public void close(){
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
}

