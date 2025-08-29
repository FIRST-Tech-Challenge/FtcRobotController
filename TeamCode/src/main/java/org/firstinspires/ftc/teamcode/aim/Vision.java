package org.firstinspires.ftc.teamcode.aim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import edu.wpi.first.math.MathUtil;

public class Vision {
    private Limelight3A camera;
    private Telemetry telemetry;
    private boolean isDataOld = false;
    private LLResult result;
    double cameraHeight;
    double cameraAngle;
    double targetHeight;

    public void init(final HardwareMap hardwareMap, Telemetry telemetry, int pipeline,
                     String cameraName, double cameraHeight, double cameraAngle,
                     double targetHeight) {
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.targetHeight = targetHeight;

        camera = hardwareMap.get(Limelight3A.class, cameraName);
        this.telemetry = telemetry;
        //camera.setPollRateHz(50);
        camera.pipelineSwitch(pipeline);
    }

    public void start() {
        camera.start();
    }

    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    private double[] calTargetPosition(double tx, double ty) {
        // Adjust ty for camera mount angle
        double tyAdjusted = ty + this.cameraAngle;

        // Height difference (camera is ABOVE target)
        double heightDiff = this.cameraHeight - this.targetHeight;  // This will be positive

        // Calculate distance - use absolute value of tyAdjusted since target is below
        double distance;
        if (Math.abs(tyAdjusted) > 0.1) {  // Avoid division by zero
            distance = heightDiff / Math.tan(Math.toRadians(Math.abs(tyAdjusted)));
        } else {
            // Target is at same level as camera - use area method or set large distance
            distance = 100.0;  // Default distance
        }

        // Calculate relative position
        double relativeX = distance * Math.tan(Math.toRadians(tx));
        double relativeY = distance;

         return new double[]{relativeX, relativeY};
    }

    public double[] getTargetPosition() {
        return calTargetPosition(this.getTx(0), this.getTy(0));
    }

    public double getTargetForwardOffset() {
        double[] pos = getTargetPosition();
        return pos[1];
    }

    public double getTargetStrafeOffset() {
        double[] pos = getTargetPosition();
        return pos[0];
    }

    public void update() {
        LLResult tmpResult = camera.getLatestResult();
        if (tmpResult != null && tmpResult.isValid()) {
            result = tmpResult;
            long staleness = result.getStaleness();
            // Less than 100 milliseconds old
            // TODO: could check this before assigning tmpResult to result
            isDataOld = staleness >= 100;
        }
    }
}