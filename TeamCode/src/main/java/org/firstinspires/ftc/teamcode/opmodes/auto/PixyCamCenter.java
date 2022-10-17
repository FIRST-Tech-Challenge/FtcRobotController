package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Vuforia;
@Autonomous (name = "pixycam testing", group = "Autonomous")
public class PixyCamCenter extends OpMode {

    protected PixyCam pixycam;
    private boolean stopRequested;
    private PixyCam.Block block;


    /** Initialization */
    public void init(){
        stopRequested = false;
        // Timeouts to determine if stuck in loop
        this.msStuckDetectInit     = 20000;
        this.msStuckDetectInitLoop = 20000;
        // Initialize motors
        pixycam = new PixyCam(hardwareMap.get(I2cDeviceSynch.class, "sensor_color"));




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
        String s = block.width + " " + block.height;
        String coords = block.x + ", " + block.y;
        telemetry.addData("block", s);
        telemetry.addData("coords", coords);
    }


    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}