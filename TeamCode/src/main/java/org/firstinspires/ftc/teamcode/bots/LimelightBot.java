package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.sample.Sample;

import java.util.ArrayList;
import java.util.List;

public class LimelightBot extends GyroBot {

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


