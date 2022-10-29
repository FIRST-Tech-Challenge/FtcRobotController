package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Autonomous (name = "pixycam testing", group = "Autonomous")
public class PixyCamCenter extends BaseOpMode {

    protected PixyCam pixycam;
    private boolean stopRequested;
    private PixyCam.Block block;


    /** Initialization */
    public void init(){
        super.init();
        stopRequested = false;
        // Timeouts to determine if stuck in loop
        // Initialize motors
        pixycam = hardwareMap.get(PixyCam.class, "pixy");


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
        block = pixycam.GetBiggestBlock(PixyCam.BLUE);
        Log.d("block ", block.toString());
        String s = block.width + " " + block.height;
        String coords = block.x + ", " + block.y;
        int rotationOffset = pixycam.headingOffset(PixyCam.BLUE);
        int distanceOffset = pixycam.distanceOffset(20);
        telemetry.addData("rotationOffset", rotationOffset);
        telemetry.addData("block", s);
        telemetry.addData("coords", coords);
        Log.d("rotationOffset", rotationOffset + " ");
        telemetry.update();
        if(rotationOffset > 20){
            driveSystem.turn(60, 0.5);
        }

        if(rotationOffset < -20){
            driveSystem.turn(-60, 0.5);
        }
        if(rotationOffset < 20 || rotationOffset > -20) {
            driveSystem.setMotorPower(0);
        }
        if(distanceOffset > 5){
            driveSystem.driveToPosition(20, DriveSystem.Direction.BACKWARD, 0.3);
        }
        else if(distanceOffset < 5){
            driveSystem.driveToPosition(20, DriveSystem.Direction.FORWARD, 0.4);
        }
        else{
            driveSystem.setMotorPower(0);
        }

    }


    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}