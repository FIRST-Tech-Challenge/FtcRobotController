package org.firstinspires.ftc.team13590;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Arrays;
import java.util.List;

public class FieldLocalization {
    private float robotWidth;
    private float robotLength;
    /**
     * Position of the farthest corner of the Blue OZ which still counts as a park
     */
    public final Position BlueOZ = new Position(DistanceUnit.INCH, -36, 53.4, 0, 0);
    /**
     * Position of the Blue Rung down the middle
     */
    public final Position BlueRung = new Position(DistanceUnit.INCH, 0, 24 + (robotLength/2), 0, 0);
    /**
     * Position of the farthest corner of the Red OZ which still counts as a park
     */
    public final Position RedOZ = new Position(DistanceUnit.INCH, 36, -54.9,  0,0);
    /**
     * Position of the Red Rung down the middle
     */
    public final Position RedRung = new Position(DistanceUnit.INCH, 0, -24 - (robotLength/2), 0, 0);

    /**
     * Position of the closest Blue Sample to the center of the field
     */
    public Position firstBlueSample;
    /**
     * Position of the closest Red Sample to the center of the field
     */
    public Position firstRedSample;
    /**
     * Position of the closes Blue Neutral Sample to the center of the field
     */
    public Position firstBlueNeutral;
    /**
     * Position of the closes Red Neutral Sample to the center of the field
     */
    public Position firstRedNeutral;
    /**
     * variable used internally to determine team color
     */

    // Measurement from middle of rung to side sub wall
    private final float rungHalfWidth = 13.75f;
    // Measurement from sub wall to the farthest place you can score high rung from
    private final float rungFullLength = 24f;
    private final Position blueRungAREA1 = new Position(DistanceUnit.INCH, BlueRung.x + rungHalfWidth, BlueRung.y, 0, 0);
    private final Position blueRungAREA2 = new Position(DistanceUnit.INCH, BlueRung.x - rungHalfWidth, BlueRung.y + rungFullLength, 0,0);


    private final Position redRungAREA1 = new Position(DistanceUnit.INCH, RedRung.x + rungHalfWidth, RedRung.y, 0, 0);
    private final Position redRungAREA2 = new Position(DistanceUnit.INCH, RedRung.x - rungHalfWidth, RedRung.y + rungFullLength, 0,0);

    private final Position blueOzAREA2 = new Position(DistanceUnit.INCH, -36, 60, 0,0);
    private final Position blueOzAREA1 = new Position(DistanceUnit.INCH, -72 + robotWidth/2, 72 - robotLength/2, 0,0);

    private final Position redOzAREA2 = new Position(DistanceUnit.INCH, 36, -60, 0,0);
    private final Position redOzAREA1 = new Position(DistanceUnit.INCH, 72 - robotWidth/2, -72 + robotLength/2, 0,0);


    private final Position blueHsubAREA1 = new Position(DistanceUnit.INCH, 24, -24 + robotWidth/2, 0,0);
    private final Position blueHsubAREA2 = new Position(DistanceUnit.INCH, 12 + robotLength/2, 24 - robotWidth/2, 0,0);

    private final Position redHsubAREA1 = new Position(DistanceUnit.INCH, -12 - robotLength/2, -24 + robotWidth/2, 0,0);
    private final Position redHsubAREA2 = new Position(DistanceUnit.INCH, -24, 24 - robotWidth/2, 0,0);


    private RobotHardware robotObj;
    private boolean teamColorBlue;

    private enum fieldAreas {
        observationZone,
        rungs,
        submersible,
        hangableSubmersible,
        blank
    }

    /**
     * Class constructor; initialize when you detect the first april tag (should be an april tag on your team's side)
     * @param detectedTagID The April Tag you detected.
     * @param robotObject Your RobotHardware object you first created at the beginning of the class.
     *                    Kinda like giving the class your credit card info so it can make autonomous purchases w/o
     *                    your permission, except now its just able to move your robot. (they're watching...) :)
     */
    public FieldLocalization (int detectedTagID, RobotHardware robotObject){
        List<Integer> blueAPTList = Arrays.asList(11, 12, 13);
        List<Integer> redAPTList = Arrays.asList(14, 15, 16);

        if (redAPTList.contains(detectedTagID))
        {
            teamColorBlue = true;
        }
        else if (blueAPTList.contains(detectedTagID))
        {
            teamColorBlue = false;
        }

        robotObj = robotObject;
        robotLength = robotObject.ROBOT_LENGTH;
        robotWidth = robotObject.ROBOT_WIDTH;
    }

    /**
     * One of the main Assistant-type methods you should be calling when using this class.
     * @param currentPos The current position of the robot needed to check when to assist in moving the elbow.
     */
    public void elbowAssistant(Pose3D currentPos, double heading){
        fieldAreas area = poseChecker(currentPos);
        if (area == fieldAreas.rungs){
            robotObj.elbowDrive.setTargetPosition(robotObj.elbowTrigPosition(currentPos, heading));
        } else if (area == fieldAreas.submersible ||
                area == fieldAreas.hangableSubmersible) {
            if (heading + 90 >= 0) {
                robotObj.elbowDrive.setTargetPosition((int) (robotObj.ELBOW_PARALLEL));
            } else { robotObj.elbowDrive.setTargetPosition((int) (robotObj.ELBOW_BACKWARD_PARALLEL));}
        }

        robotObj.myOpMode.telemetry.addData(area.toString(), ": current area");
    }

    public int elbowAssistantPassive(Pose3D currentPos, double heading){
        int elbowPos = robotObj.elbowDrive.getCurrentPosition();
        fieldAreas area = poseChecker(currentPos);
        if (area == fieldAreas.rungs){
            elbowPos = (robotObj.elbowTrigPosition(currentPos, heading));
        } else if (area == fieldAreas.submersible) {
            if (heading + 90 >= 45 && heading + 90 <= 135) {
                elbowPos = (int) (robotObj.armByExtender());
            } else {
                elbowPos = (int) (robotObj.ELBOW_PARALLEL);
            }
        } else if (area == fieldAreas.hangableSubmersible) {
            elbowPos = (int) robotObj.ELBOW_PERPENDICULAR;
        } else {
            robotObj.myOpMode.telemetry.addData("Area not partnered with a position", "...");
        }

        robotObj.myOpMode.telemetry.addData(area.toString(), ": current area");
        return elbowPos;
    }


    private fieldAreas poseChecker(Pose3D currentPos){
        if (teamColorBlue){
            return bluCheckerFrag(currentPos);
        } else {
            return redCheckerFrag(currentPos);
        }

    }

    private fieldAreas bluCheckerFrag(Pose3D currentPos){
        // translate BotPose object into Position object
        Position pendingPos = currentPos.getPosition();
        if (withinArea(blueRungAREA1, blueRungAREA2, pendingPos)) {
            return fieldAreas.rungs;
        } else if (withinArea(blueOzAREA1, blueOzAREA2, pendingPos)) {
            return fieldAreas.observationZone;
        } else if (withinArea(blueHsubAREA1, blueHsubAREA2, pendingPos)) {
            return fieldAreas.hangableSubmersible;
        } else if (withinArea(redHsubAREA1, redHsubAREA2, pendingPos)) {
            return fieldAreas.submersible;
        } else {
            return fieldAreas.blank;
        }
    }

    private fieldAreas redCheckerFrag(Pose3D currentPos){
        // translate BotPose object into Position object
        Position pendingPos = currentPos.getPosition();
        if (withinArea(redRungAREA1, redRungAREA2, pendingPos)) {
            return fieldAreas.rungs;
        } else if (withinArea(redOzAREA1, redOzAREA2, pendingPos)) {
            return fieldAreas.observationZone;
        } else if (withinArea(redHsubAREA1, redHsubAREA2, pendingPos)) {
            return fieldAreas.hangableSubmersible;
        } else if (withinArea(blueHsubAREA1, blueHsubAREA2, pendingPos)) {
            return fieldAreas.submersible;
        } else {
            return fieldAreas.blank;
        }
    }

    /**
     * Pass in positions like if you were drawing a rectangle with 2 points; NO ANGLES. For more complex polygons, nag me
     * to write it up
     * @param corner1 should be the corner with the lowest x and greatest y
     * @param corner2 should be the corner with the highest x and lowest y
     * @param pendingPos position you want to check
     */
    private boolean withinArea(Position corner1, Position corner2, Position pendingPos){
        boolean withinX = ( corner1.x >= pendingPos.x) && (pendingPos.x >= corner2.x);
        boolean withinY = ( corner1.y <= pendingPos.y) && (pendingPos.y <= corner2.y);

        return withinX && withinY;
    }
}
