package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.sample.Sample;

import java.util.ArrayList;
import java.util.List;

public class LimelightBot extends PinchBot {

    public LLResult result = null;
    public Pose3D botpose = null;
    public Limelight3A limelight;
    public boolean inAutoPickup = false;
    public static final double targetTx = 5f;
    public static final double targetTy = 8f;


    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

    }

    public LimelightBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void alignWithSampleJulian(boolean isBlueAlliance, boolean includeSharedSample, boolean isSpecimen, ElapsedTime scanTimer) {
        if (inAutoPickup){
            // already in auto pickup mode
            return;
        }
        Sample sample = detectOne(isBlueAlliance, includeSharedSample);
        if (sample == null) {
            return;
        }
        scanTimer.reset();

        double xThreshold = 2;
        double yThreshold = 2;
        boolean isXCloseEnough = Math.abs(sample.getDeltaX() ) < xThreshold;
        boolean isYCloseEnough = Math.abs(sample.getDeltaY() ) < yThreshold;

        int delta = (int)Math.round(sample.getDeltaY() * 7);
        telemetry.addData("extendSlide-------->", delta);
        moveSlideByDelta(delta);

        double distance = sample.getDeltaX() * 2.2;
        telemetry.addData("DRIVE --------------> distance :", distance);
        strafing(distance);
        // sample is close enough, pick it up
        inAutoPickup = true;
        // rotate to the sample orientation
        rotateToAngle(sample.getSampleAngle());
        // open the pinch
        openPinch();
        // lower the pivot

        schedule(this::stopCoordinateDrive, 1000);

//        `if (isSpecimen) {
//            schedule(this::pivotToPickupPosSpecimen, 1500);
//        }
//        else {
//            schedule(this::pivotToPickupPosSample, 1500);
//        }
//        // close the pinch at a future time
//        schedule(this::closePinch, 2000);
//        // raise the pivot at a future time and also reset the auto pickup mode
//        schedule(this::pivotToPickupUpPos, 2500);`

        /*if (isXCloseEnough && isYCloseEnough) {
            strafing(0);
            // sample is close enough, pick it up
            inAutoPickup = true;
            // rotate to the sample orientation
            rotateToAngle(sample.getSampleAngle());
            // open the pinch
            openPinch();
            // lower the pivot
            pivotToPickupPos(isSpecimen);
            // close the pinch at a future time
            schedule(this::closePinch, 500);
            // raise the pivot at a future time and also reset the auto pickup mode
            schedule(this::pivotToPickupUpPos, 1000);
        }
        else{
            // sample is not close enough, move to the sample
            pivotToSearchPos();
            if (!isYCloseEnough && scanTimer.time() > 0.25) {
                // extend/retract the slide based on delta Y
                int delta = (int)Math.round(sample.getDeltaY() * 5);
                telemetry.addData("extendSlide-------->", delta);
                moveSlideByDelta(delta);
                scanTimer.reset();
            }
            if (!isXCloseEnough && scanTimer.time() > 0.25) {
                strafing(0);
                // move the robot sideways based on delta X * 0.26875
                double distance = sample.getDeltaX() * -3;
                telemetry.addData("DRIVE --------------> distance :", distance);
                strafing(distance);
                scanTimer.reset();
            }
        }*/
    }

    /**
     * Align with sample without strafe the robot,
     *  given the pinch arm is installed in the center of rotation arm, we can calculate the center of pinch arm with detected sample angle
     *  the pinch arm center is in a range of error, we can pinch
     * @param isBlueAlliance
     * @param includeSharedSample
     * @param isSpecimen
     */
    public void alignWithSample(boolean isBlueAlliance, boolean includeSharedSample, boolean isSpecimen) {
        if (inAutoPickup){
            // already in auto pickup mode
            return;
        }
        Sample sample = detectOne(isBlueAlliance, includeSharedSample);
        if (sample == null || sample.isLLResultValid() < 0) {
            // no sample found, do nothing
            telemetry.addData("Sample not found : ", sample);
            return;
        }
        Sample.OptimalDeltaY result = sample.calculateOptimalDeltaY(10.0);
        if (result.isReachable){
            telemetry.addData("Sample is reachable : ", result.toString());
            // rotate to angle
            rotateToAngle(result.rotationAngle);
            // adjust slide
            moveSlideByDelta(result.deltaY);
        }
        else{
            // do nothing, the sample is not reachable
            telemetry.addData("Sample is not reachable : ", result.toString());
        }

    }
    public void pickup(boolean isSpecimen){

        openPinch();
        // lower the pivot
        if (isSpecimen){
            pivotToPickupPosSpecimen();
        }
        else {
            pivotToPickupPosSample();
        }
        // close the pinch at a future time
        schedule(this::closePinch, 500);
        // raise the pivot at a future time and also reset the auto pickup mode
        schedule(this::pivotToPickupUpPos, 1000);

    }
    public Sample detectOne(boolean isBlueAlliance, boolean includeSharedSample) {
        List<Integer> pipelines = new ArrayList<>();
        if (includeSharedSample) {
            // TODO : disable Shared detection for now
            // pipelines.add(Sample.sharedSamplePipeline);
        }
        if (isBlueAlliance) {
            pipelines.add(Sample.blueSamplePipeline);
        } else {
            pipelines.add(Sample.redSamplePipeline);
        }

        List<Sample> results = new ArrayList<>();
        for (int pipeline : pipelines) {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
            // TODO : need to wait for the pipeline to switch
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                // ignore
            }
            telemetry.addData("pipeline==============", pipeline);
            for (int i = 0; i <3; i++) {
                // try 3 times for each pipeline
                LLResult result = limelight.getLatestResult();
                if (result == null){
                    telemetry.addData("LL Result is null : ", i);
                    continue;
                }
                Sample sample = new Sample(result, targetTx, targetTy);
                telemetry.addData("result<<<<<<<<<<<<<<<<<.ta", result.getTa());
                telemetry.addData("result >>>>>>>>>>>>>>>>> ", sample.toString());
                if (sample.isLLResultValid() > 0) {
                    results.add(sample);
                    break;
                }
            }
        }
        //telemetry.addData("pipeline.results", results.size());
        // find the sample closest to the target on XY plane
        Sample closest = null;
        double minDistance = Double.MAX_VALUE;
        for (Sample sample : results) {
            double distance = sample.distanceXY();
            telemetry.addData("pipeline::distance", distance);
            if (distance < minDistance) {
                minDistance = distance;
                closest = sample;
            }
        }
        return closest;

    }

    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
}
