package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LimeLightWrapper {
    Limelight3A limelight;
    //The position of each april tag on the field 
    public static final Vector2d[] APRIL_TAG_POSITIONS = new Vector2d[]{
            //11
            new Vector2d(-72,-48),
            //12
            new Vector2d(0,72),
            //13
            new Vector2d(72,-48),
            //14
            new Vector2d(72,48),
            //15
            new Vector2d(0,-72),
            //16
            new Vector2d(-72,48)

    };

    public LimeLightWrapper(Limelight3A limelight3A) {
        limelight = limelight3A;
    }
    //starts up the limelight with the first pipeline
    public void start() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    //starts up the limelight with a set pipeline
    public void start(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }
//gets all the valid inputs that the limelight finds
    public LLResult getVaildResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //only returns good results 
                return result;
            }
        }
        return null;
    }
//gets the distance using just the botpose
    public Pose3D distanceFromTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }
//gets the distance using the botpose and the yaw
    public Pose3D distanceFromTag(double yaw) {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(yaw);
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }
    //takes a pose3d from a distance from the tag and localizes it based on which April tag it is
    public Pose3D localize(int i,Pose3D pose3D) {
        Vector2d vector2d = APRIL_TAG_POSITIONS[i-11];
        double x = vector2d.x+pose3D.getPosition().x;
        double y = vector2d.y+pose3D.getPosition().y;
        double z = 0;
        return new Pose3D(new Position(DistanceUnit.INCH,x,y,z,pose3D.getPosition().acquisitionTime),pose3D.getOrientation());
    }


}
