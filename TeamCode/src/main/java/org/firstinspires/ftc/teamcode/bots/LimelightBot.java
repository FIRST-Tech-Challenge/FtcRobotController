package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.sample.Sample;

import java.util.ArrayList;
import java.util.List;

public class LimelightBot extends PinchBot {

    public LLResult result = null;
    public Pose3D botpose = null;
    public Limelight3A limelight;

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);

        limelight.start();


    }

    public LimelightBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void pickup(boolean isBlueAlliance, boolean includeSharedSample) {
        Sample sample = detectOne(isBlueAlliance, includeSharedSample);
        // rotate to the sample orientation
        rotateToAngle(sample.getSampleAngle());
        double xThreshold = 0.5;
        double yThreshold = 0.5;
        boolean isXCloseEnough = Math.abs(sample.getDeltaX() ) < xThreshold;
        boolean isYCloseEnough = Math.abs(sample.getDeltaY() ) < yThreshold;
        if (isXCloseEnough && isYCloseEnough) {
            // sample is close enough, pick it up
            // open the pinch
            openPinch();
            // lower the pivot
            pivotToPickupPos();
            // close the pinch at a future time
            closePinchInTime(2000);
            // raise the pivot at a future time
            pivotToUpPosInTime(3000);
        }
        else{
            // sample is not close enough, move to the sample
            pivotToSearchPos();
            if (!isYCloseEnough) {
                // extend/retract the slide based on delta Y
                boolean extendSlide= sample.getDeltaY() > 0;
                slideControl(extendSlide, !extendSlide);
            }
            if (!isXCloseEnough) {
                // move the robot sideways based on delta X
                int direction = sample.getDeltaX() > 0 ? DIRECTION_RIGHT : DIRECTION_LEFT;
                double distance = sample.getDeltaX() * 0.5;
                driveStraightByDistance(direction, distance, 0.3);
            }
        }
    }
    public Sample detectOne() {
        return detectOne(false, false);
    }

    public Sample detectOne(boolean isBlueAlliance, boolean includeSharedSample) {
        List<Integer> pipelines = new ArrayList<>();
        if (isBlueAlliance) {
            pipelines.add(Sample.blueSamplePipeline);
        } else {
            pipelines.add(Sample.redSamplePipeline);
        }
        if (includeSharedSample) {
            pipelines.add(Sample.sharedSamplePipeline);
        }

        List<Sample> results = new ArrayList<>();
        for (int pipeline : pipelines) {
            limelight.pipelineSwitch(pipeline);
            for (int i = 0; i <3; i++) {
                // try 3 times for each pipeline
                LLResult result = limelight.getLatestResult();
                Sample sample = new Sample(result);
                if (sample.isLLResultValid() > 0) {
                    results.add(sample);
                    break;
                }
            }
        }
        // find the sample closest to the target on XY plane
        Sample closest = null;
        double minDistance = Double.MAX_VALUE;
        for (Sample sample : results) {
            double distance = sample.distanceXY();
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


