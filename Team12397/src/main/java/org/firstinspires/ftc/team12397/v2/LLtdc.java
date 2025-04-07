package org.firstinspires.ftc.team12397.v2;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// LimeLight Trigonometry Distance Calculator
public class LLtdc {
    private Limelight3A limelight;
    private TdcReturnObject returnObject;

    // absolute parameters: all TBD at 4/6/2025 ---
    /**
     * TBD; based on robot build
     * The height from the lens to the floor
     *
     */
    private final double lensHeightIN = 0;
    /**
     * TBD; based on robot build
     * The parallel distance from the lens to the center of the robot
     * (parallel distance as in: shortest distance to the robot's front plane's center line.)
     */
    private final double lensOffsetIN = 0;
    private final double armMaximumReachIN = 0;

    // measured parameters ---
    private double yPlaneRads;
    private double xPlaneRads;

    // calculated parameters ---
    /**
     * how far forward/backward the robot must move
     */
    private double yCorrection;
    /**
     * how far right/left the robot must move
     */
    private double xCorrection;
    /**
     * how cw/ccw the robot must turn
     */
    private double yawCorrection;
    /**
     * how far the arm must extend/retract
     */
    private double armCorrection;
    public LLtdc(Limelight3A LL){
        limelight = LL;
    }

    /**
     * Starts up the limelight processor & camera. Requires a handful of seconds before running a process.
     * @param telemetry Telemetry object used in OpMode to communicate with driver team. (safety)
     * @see Telemetry
     */
    public void initialize(Telemetry telemetry){
        if (!limelight.isConnected()){
            telemetry.addLine("!!LIMELIGHT NOT CONNECTED!!\n " +
                    "Any following LL operations will return null.\n" +
                    "This class will NOT shutdown to prevent runtime errors,\n" +
                    "This class WILL go dormant and abstain from contributing data/returns.");
            limelight.stop(); // safety
            limelight.shutdown();
        } else if (!limelight.isRunning()){
            limelight.start(); // start LL & switch to object detection pipeline
            limelight.pipelineSwitch(6);
        }
    }

    /**
     * Sleeps the limelight processor & camera. Has a timed delay before starting up again.
     */
    public void sleep(){
        limelight.stop();
    }

    /**
     * PERMANENTLY shuts down the limelight connection. This process cannot be undone unless
     * the robot/connection is reset.
     */
    public void shutdown(){
        limelight.shutdown();
    }


    private void formulateYcorrection(){
        yCorrection = yPlaneRads; // placeholder! please chill until a later date... (insert formula & continue here)
    }

    private void fabricateReturnObject(){
        // clawYawCorrection will hold 0 until further build details are released.
        returnObject = new TdcReturnObject(yawCorrection, xCorrection, yCorrection, armCorrection, 0);
    }

    public TdcReturnObject getTdcReturn() {
        return returnObject;
    }
}
