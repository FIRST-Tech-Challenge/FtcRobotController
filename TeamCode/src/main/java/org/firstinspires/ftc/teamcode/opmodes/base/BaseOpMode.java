package org.firstinspires.ftc.teamcode.opmodes.base;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IMUSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;

import java.util.EnumMap;

/**
 * Basic OpMode template
 */
public abstract class BaseOpMode extends OpMode {

    protected DriveSystem driveSystem;
    protected Vuforia vuforia;
    private boolean stopRequested;

    /** Initialization */
    public void init(){
        stopRequested = false;
        // Timeouts to determine if stuck in loop
        this.msStuckDetectInit = 20000;
        this.msStuckDetectInitLoop = 20000;
        // Initialize motors
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap);
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

    @Override
    public void stop() {
        stopRequested = true;
        super.stop();
    }
}
