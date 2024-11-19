package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LimeLightWrapper implements LocalizerInterface{

    /*Will be used later on to have Limelight report data
    As the starting wall to be pointed Negative
    This saves the hassle of creating new nearly identical
    sets of autos to switch between blue and red
    */
    public enum Color {
        RED_SIDE,
        BLUE_SIDE
    }
    private Color currentColor;

    public final static double M_TO_IN = 39.3700787402;
    public double weight = 0.33;

    Limelight3A limelight;

    public LimeLightWrapper(Limelight3A limelight3A) {
        limelight = limelight3A;
        start();
    }
    //starts up the limelight with the first pipeline
    public void start() {
        limelight.setPollRateHz(100);
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
    public int getAprilTagId(LLResultTypes.FiducialResult fiducialResult) {
        return fiducialResult.getFiducialId();
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
        limelight.updateRobotOrientation(yaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    //takes a pose3d from a distance from the tag and localizes it based on which April tag it is
    public static Pose3D MtoIN(Pose3D pose3D) {
        double x = (pose3D.getPosition().x*M_TO_IN);
        double y = (pose3D.getPosition().y*M_TO_IN);
        double z = (pose3D.getPosition().z*M_TO_IN);

        return new Pose3D(new Position(DistanceUnit.INCH,x,y,z,pose3D.getPosition().acquisitionTime),pose3D.getOrientation());
    }


    /**
     * @return [double] Returns the weight associated with this localizer
     */
    @Override
    public double getWeight() {
        return weight;
    }

    /**
     * @return [Pose2d] Returns localization data based on AprilTags
     */
    @Override
    public Pose2d getPosition() {
        double x = 0, y = 0, h = 0;
        LLResult result = getVaildResult();
        if (result != null){
            Pose3D botpose = MtoIN(result.getBotpose());
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                h = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            }
            if(currentColor==Color.BLUE_SIDE) {
                x *= -1;
                y *= -1;
                h *= -1;
            }
        }
        return new Pose2d(x,y,h);
    }

    public Pose2d getPosition(double yaw) {
        double x = 0, y = 0, h = 0;
        LLResult result = getVaildResult();
        limelight.updateRobotOrientation(yaw);
        if (result != null){
            Pose3D botpose = MtoIN(result.getBotpose_MT2());
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                h = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            }
            if(currentColor==Color.RED_SIDE) {
                x *= -1;
                y *= -1;
                h *= -1;
            }
        }
        return new Pose2d(x,y,h);
    }

    @Override
    public boolean isValid() {
        LLResult result = getVaildResult();
        return result != null && result.getBotpose() != null;
    }

    public void setColor(Color color){
        currentColor = color;
    }

    //Do Nothing
    @Override
    public void setInitialPosition(Pose2d pose) {}

}
