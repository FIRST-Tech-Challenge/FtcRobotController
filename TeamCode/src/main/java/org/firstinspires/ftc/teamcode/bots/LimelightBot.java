package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public void pickup(boolean isBlueAlliance, boolean includeSharedSample, boolean isSpecimen, Telemetry telemetry) {
        if (inAutoPickup){
            // already in auto pickup mode
            return;
        }
        Sample sample = detectOne(isBlueAlliance, includeSharedSample, telemetry);
        if (sample == null) {
            return;
        }
        double xThreshold = 2;
        double yThreshold = 2;
        boolean isXCloseEnough = Math.abs(sample.getDeltaX() ) < xThreshold;
        boolean isYCloseEnough = Math.abs(sample.getDeltaY() ) < yThreshold;
        if (isXCloseEnough && isYCloseEnough) {
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
            if (!isYCloseEnough) {
                // extend/retract the slide based on delta Y
                int delta = (int)Math.round(sample.getDeltaY() * 5);
                if (telemetry != null) telemetry.addData("extendSlide-------->", delta);
                slideByDelta(delta);
            }
            if (!isXCloseEnough) {
                // move the robot sideways based on delta X
                double power = sample.getDeltaX() * 0.1;
                if (telemetry != null) telemetry.addData("DRIVE --------------> power :", power);
                strafing(power);
            }
        }
    }

    public Sample detectOne(boolean isBlueAlliance, boolean includeSharedSample, Telemetry telemetry) {
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
            if (telemetry != null) telemetry.addData("pipeline==============", pipeline);
            for (int i = 0; i <3; i++) {
                // try 3 times for each pipeline
                LLResult result = limelight.getLatestResult();
                if (telemetry != null) telemetry.addData("result<<<<<<<<<<<<<<<<<.ta", result.getTa());
                Sample sample = new Sample(result, targetTx, targetTy);
                if (telemetry != null) telemetry.addData("result >>>>>>>>>>>>>>>>> ", sample.toString());
                if (sample.isLLResultValid() > 0) {
                    results.add(sample);
                    break;
                }
            }
        }
        //if (telemetry != null) telemetry.addData("pipeline.results", results.size());
        // find the sample closest to the target on XY plane
        Sample closest = null;
        double minDistance = Double.MAX_VALUE;
        for (Sample sample : results) {
            double distance = sample.distanceXY();
            if (telemetry != null) telemetry.addData("pipeline::distance", distance);
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
