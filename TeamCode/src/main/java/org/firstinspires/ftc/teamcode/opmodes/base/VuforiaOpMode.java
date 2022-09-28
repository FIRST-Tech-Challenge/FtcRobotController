package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;

import java.util.EnumMap;

/**
 * Basic OpMode template
 */
@Autonomous(name = "Vuforia Signal Sleeve", group = "Autonomous")
public class VuforiaOpMode extends OpMode {

    protected DriveSystem driveSystem;
    protected Vuforia vuforia;
    private boolean stopRequested;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    /** Initialization */
    public void init(){
        stopRequested = false;
        // Timeouts to determine if stuck in loop
        this.msStuckDetectInit     = 20000;
        this.msStuckDetectInitLoop = 20000;
        // Initialize motors
        setCamera(CameraChoice.WEBCAM1);


    }

    /** Initialize Vuforia object with given camera
     * @param cameraChoice
     */
    protected void setCamera(CameraChoice cameraChoice) {
        vuforia = new Vuforia(hardwareMap, cameraChoice);
    }

    /** Returns if a stop has been requested or if execution is
     */
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }
    public void loop(){
        vuforia.identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        vuforia.identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        vuforia.identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        vuforia.identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        telemetry.addData("is a target visible? : ",  vuforia.isAnyTargetVisible());
        telemetry.addData("Visible Target: ", vuforia.nameOfTargetVisible());
        telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());
        telemetry.update();
    }


    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}
