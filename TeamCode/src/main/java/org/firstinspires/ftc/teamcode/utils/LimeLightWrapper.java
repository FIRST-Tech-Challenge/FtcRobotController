package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLightWrapper {
    Limelight3A limelight;
    public static final Vector2d[] APRIL_TAG_POSITIONS = new Vector2d[]{

    };

    public LimeLightWrapper(Limelight3A limelight3A) {
        limelight = limelight3A;
    }

    public void start() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void start(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public LLResult getVaildResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result;
            }
        }
        return null;
    }

    public Pose3D distanceFromTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    public Pose3D distanceFromTag(double yaw) {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(yaw);
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }
    public Pose3D localize(int i,Pose3D pose3D) {
        Vector2d vector2d = APRIL_TAG_POSITIONS[i];
        double x = vector2d.x+pose3D.getPosition().x;
        double y = vector2d.y+pose3D.getPosition().y;
        double z = 0;
        return new Pose3D(new Position(DistanceUnit.INCH,x,y,z,pose3D.getPosition().acquisitionTime),pose3D.getOrientation());
    }


}
