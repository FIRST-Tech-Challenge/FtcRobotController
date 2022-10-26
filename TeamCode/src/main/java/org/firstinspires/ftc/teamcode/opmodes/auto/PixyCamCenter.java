package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Vuforia;
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
        block = pixycam.GetBiggestBlock(3);
        Log.d("block ", block.toString());
        String s = block.width + " " + block.height;
        String coords = block.x + ", " + block.y;
        int offset = pixycam.offSetX();
        telemetry.addData("offset", offset);
        telemetry.addData("block", s);
        telemetry.addData("coords", coords);
        Log.d("offset", offset + " ");
        telemetry.update();
        if(offset > 20){
            driveSystem.turn(60, 0.5);
        }

        if(offset < -20){
            driveSystem.turn(-60, 0.5);
        }
        if(offset < 20 || offset > -20){
            driveSystem.setMotorPower(0);
        }
    }


    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}