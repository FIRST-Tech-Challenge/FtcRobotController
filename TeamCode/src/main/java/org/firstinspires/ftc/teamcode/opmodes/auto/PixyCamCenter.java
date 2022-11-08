package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Autonomous (name = "pixy cam", group = "Autonomous")
public class PixyCamCenter extends BaseOpMode {

    protected PixyCam pixycam;
    private boolean stopRequested;
    private PixyCam.Block block;
    private int count;


    /** Initialization */
    public void init(){
        super.init();
        stopRequested = false;
        // Timeouts to determine if stuck in loop
        // Initialize motors
        pixycam = hardwareMap.get(PixyCam.class, "pixy");
        count = 0;


    }

    /** Initialize Vuforia object with given camera
     * @param cameraChoice
     */

    /** Returns if a stop has been requested or if execution is
     */
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }
    public void loop(){
        block = pixycam.GetBiggestBlock(PixyCam.YELLOW);
        Log.d("block ", block.toString());
        String s = block.width + " " + block.height;
        String coords = block.x + ", " + block.y;
        int rotationOffset = pixycam.headingOffset(PixyCam.YELLOW);
        int distanceOffset = pixycam.distanceOffset(PixyCam.YELLOW, 50);
//        telemetry.addData("block", s);
//        telemetry.addData("coords", coords);
        telemetry.addData("distanceOFfset", distanceOffset);
        telemetry.addData("rotationOffset", rotationOffset);
        Log.d("rotationOffset", rotationOffset + " ");
        Log.d("distanceOfset", distanceOffset + " ");
        telemetry.update();
        if(gamepad1.a){
            align(PixyCam.YELLOW, 40);
        }

    }


    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}