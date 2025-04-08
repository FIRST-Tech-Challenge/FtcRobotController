package org.firstinspires.ftc.team12397.v2;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

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
    private final double lensHeightIN = 4.75- 1.75;
    /**
     * TBD; based on robot build
     * The parallel distance from the lens to the center of the robot
     * (parallel distance as in: shortest distance to the robot's front plane's center line.)
     */
    private final double lensOffsetIN = 3.875;
    /**
     *TBD; based on robot build
     * The distance from the lens to the front plane of the robot
     */
    private final double lensDepthOffsetIN = 0;
    private final double armMaximumReachIN = 12;

    // measured parameters ---
    private double yPlaneRads;
    private double xPlaneRads;

    // calculated parameters ---
    private double rawYcorrection;

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

    private boolean scanSuccessful;
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

    public void assessEnvironment(int recursionDepth){
        limelight.captureSnapshot("try1");
        List<LLResultTypes.DetectorResult> detectorResults = limelight.getLatestResult().getDetectorResults();

        if (!detectorResults.isEmpty()) { scanSuccessful = true;
            // filter out unreliable guesses
            for (int i = detectorResults.size()-1; i >= 0; i--) {
                if (detectorResults.get(i).getConfidence() < 85) {
                    detectorResults.remove(i);
                }
            }

            /*
            // filter out by color (needs to be figured out)
            // Run obd & then 2 clr pipelines?
            // cv2 to find pxl clr?
            for (int i = detectorResults.size()-1; i >= 0; i--) {
                if (detectorResults.get(i).getConfidence() < 85) {
                    detectorResults.remove(i);
                }
            }
             */

            // get the closest one (by area)
            LLResultTypes.DetectorResult closest = detectorResults.get(0);
            for (int i = detectorResults.size()-1; i > 0; i--) {
                if (detectorResults.get(i).getTargetArea() > closest.getTargetArea()){
                    closest = detectorResults.get(i);
                }
            }

            yPlaneRads = Math.toRadians(closest.getTargetYDegrees());
            xPlaneRads = Math.toRadians(closest.getTargetXDegrees());


        } else {
            recursionDepth--;
            scanSuccessful = false;
            if (recursionDepth !=0) {
                assessEnvironment(recursionDepth);
            } else {
                yPlaneRads = 0;
                xPlaneRads = 0;
            }
        }
    }

    private void formulateRobotCorrections(){
        // cot(yPlaneRads) * lensHeight(opposite) = adjacent length
        rawYcorrection = (1/Math.tan(yPlaneRads)  )*lensHeightIN;
        // tan(xPlaneRads) = opposite/adjacent
        xCorrection = (Math.tan(xPlaneRads)  )*rawYcorrection;
        // -1 to make cw (+) and ccw (-): tan(yawCorrection) = (xCorrection - lensOffset) (opposite) / yCorrection (adjacent)
        yawCorrection = (Math.atan(  ((xCorrection-lensOffsetIN)*-1) /rawYcorrection));
        //
        if (rawYcorrection > armMaximumReachIN){
            // find the distance the robot must cover for the arm to be in reach.
            yCorrection = rawYcorrection - armMaximumReachIN;
            // give the rest amount of distance to the arm (armMaximumReach).
            armCorrection = rawYcorrection - yCorrection;
        } else if (rawYcorrection < armMaximumReachIN){
            // split distances between both movement mediums to achieve faster times.
            yCorrection = rawYcorrection/2;
            armCorrection = rawYcorrection/2;
        }
    }

    private void fabricateReturnObject(){
        // clawYawCorrection will hold 0 until further build details are released.
        if (scanSuccessful) {
            formulateRobotCorrections();
            returnObject = new TdcReturnObject(yawCorrection, xCorrection, yCorrection, armCorrection, 0);
        } else {
            returnObject = new TdcReturnObject(0,0,0,0,0);
        }
    }

    /**
     *
     * @return a TdcReturnObject with all calculated dimensions. If the scan was unsuccessful, all values will be 0.
     * @see TdcReturnObject
     */
    public TdcReturnObject getTdcReturn() {
        fabricateReturnObject();
        return returnObject;
    }

    public boolean getScanSuccess(){
        return scanSuccessful;
    }
}
