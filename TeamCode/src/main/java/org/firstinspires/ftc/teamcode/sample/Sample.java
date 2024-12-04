package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

public class Sample {
    public static final float targetTx = 0.2f;
    public static final float targetTy = 0.2f;
    public static final float targetArea = 15.0f;
    public static final int redSamplePipeline = 1;
    public static final int blueSamplePipeline = 2;
    public static final int sharedSamplePipeline = 0;
    public LLResult llResult;
    public LLResultTypes.ColorResult colorResult;

    // constructor
    public Sample(LLResult llResult) {
        this.llResult = llResult;
    }

    /**
     * Check if the LLResult is valid, return 1 if valid, negative if not (with different error codes
     *
     * @return
     */
    public int isLLResultValid() {

        if (llResult != null) {
            if (llResult.isValid()) {
                if (llResult.getColorResults().size() == 1) {
                    colorResult = llResult.getColorResults().get(0);
                    if (colorResult.getTargetCorners().size() > 4) {
                        return 1;
                    } else {
                        return -20 - colorResult.getTargetCorners().size();
                    }
                } else {
                    return -10 - llResult.getColorResults().size();
                }
            } else {
                return -2;
            }
        } else {
            return -1;
        }
    }

    public int getPipelineIndex() {
        return llResult.getPipelineIndex();
    }

    public String getColor() {
        switch (getPipelineIndex()) {
            case sharedSamplePipeline:
                return "Shared";
            case redSamplePipeline:
                return "Red";
            case blueSamplePipeline:
                return "Blue";
            default:
                return "Unknown";
        }
    }
    public double deltaX() {
        return llResult.getTx() - targetTx;
    }

    public double deltaY() {
        return llResult.getTy() - targetTy;
    }

    /**
     * Calculate the distance to the target on XY plane
     *
     * @return
     */
    public double distanceXY() {
        return Math.sqrt(Math.pow(deltaX(), 2) + Math.pow(deltaY(), 2));
    }
    /**
     * Calculate the delta to the target distance based on the area (distance is proportional to the square root of the area)
     *
     * @return
     */
    public double deltaDistance() {
        return Math.sqrt(targetArea) - Math.sqrt(llResult.getTa());
    }

    /**
     * Calculate the angle (in degree) of the sample based on the target corners
     *
     * @return
     */
    public int sampleAngle() {
        List<Double> p1 = colorResult.getTargetCorners().get(0);
        List<Double> p2 = colorResult.getTargetCorners().get(1);
        List<Double> p3 = colorResult.getTargetCorners().get(2);
        List<Double> p4 = colorResult.getTargetCorners().get(4);
        Double edgeP12Length = Math.sqrt(Math.pow(p1.get(0) - p2.get(0), 2) + Math.pow(p1.get(1) - p2.get(1), 2));
        Double edgeP14Length = Math.sqrt(Math.pow(p1.get(0) - p4.get(0), 2) + Math.pow(p1.get(1) - p4.get(1), 2));
        if (edgeP12Length > edgeP14Length) {
            // average of line P12 and P34
            return (int) Math.toDegrees((Math.atan2(p1.get(1) - p2.get(1), p1.get(0) - p2.get(0)) + Math.atan2(p3.get(1) - p4.get(1), p3.get(0) - p4.get(0))) / 2);
        } else {
            // average of line P14 and P23
            return (int) Math.toDegrees((Math.atan2(p1.get(1) - p4.get(1), p1.get(0) - p4.get(0)) + Math.atan2(p2.get(1) - p3.get(1), p2.get(0) - p3.get(0))) / 2);
        }
    }
    public String toString() {
        int result = isLLResultValid();
        if (result > 0) {
            return "Sample{" +
                    "x=" + deltaX() +
                    ", y=" + deltaY() +
                    ", d=" + deltaDistance() +
                    ", a=" + sampleAngle() +
                    ", c=" + getColor() +
                    '}';
        } else {
            return "Invalid LLResult with error code " + result;
        }
    }
}
