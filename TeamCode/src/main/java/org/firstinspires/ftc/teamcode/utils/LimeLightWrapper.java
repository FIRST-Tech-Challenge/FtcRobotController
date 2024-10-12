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
    private enum Color {
        RED_SIDE,
        BLUE_SIDE
    }
    private Color currentColor;

    public final double M_TO_IN = 39.3700787402;
    public double weight = 0.33;

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
    public Pose3D inToMM(Pose3D pose3D) {
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
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                h = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            }
        }
        return new Pose2d(x,y,h);
    }
}
