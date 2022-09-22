package org.firstinspires.ftc.teamcode.ebotsutil;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSize;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements.FieldTile;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.LaunchLine;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements.PlayField;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.StartLine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.TowerGoal;

import java.util.Formatter;

import static java.lang.String.format;

/**
 * Pose represents the robots position AND heading relative to field coordinates
 * It can also carry instructions on what to do after an auton travel leg
 */
public class Pose {

    /****************************************************************
    //******    INSTANCE VARIABLES
    //***************************************************************/

    private double headingDeg;         // Degrees, 0 inline with x-axis, positive CCW when viewed from top
    private FieldPosition fieldPosition;    // x, y and z position on field in inches.  0 is center of field.  x+ towards target goals.  y+ towards blue alliance side.

    @Deprecated
    private double newHeadingReadingDeg;   //New incoming reading for heading
    /***************************************************************
    //******    STATIC VARIABLES
    //****************************************************************/


    /***************************************************************
    //******    ENUMERATIONS
    //***************************************************************/
    @Deprecated
    public enum PresetPose {
        //These starting poses assume BLUE Alliance
        INNER_START_LINE(calculateXCoord(StartingSide.CAROUSEL), calculateYCoord(), 0.0)
        , OUTER_START_LINE(calculateXCoord(StartingSide.CAROUSEL), calculateYCoord(), 0.0)
        , LAUNCH_TARGET_GOAL(new LaunchLine().getX(), new TowerGoal().getY(), 0.0)
        , LAUNCH_POWER_SHOT(new LaunchLine().getX(), new TowerGoal().getY(), 0.0);

        private double xStart;
        private double yStart;
        private double headingStart;

        PresetPose(double xInput, double yInput, double headingInput){
            this.xStart = xInput;
            this.yStart = yInput;
            this.headingStart = headingInput;
        }

        public double getXStart() {
            return xStart;
        }
        public double getYStart() {
            return yStart;
        }
        public double getHeadingStart() {
            return headingStart;
        }

        private static double calculateXCoord(StartingSide startingSide){
            //Start on the bottom wall, heading = 0
            double robotHalfWidth = RobotSize.ySize.getSizeValue()/2;
            int numTiles = 0;
            if (startingSide == StartingSide.CAROUSEL) {
                numTiles = (AllianceSingleton.getAlliance()==Alliance.RED) ? 2 : 1;
            }
            double xCenter = (-FieldTile.getSize() * numTiles) + robotHalfWidth;

            return xCenter;
        }

        private static double calculateYCoord(){
            //Assumed blue alliance, robot heading 0, right wheels on start line
            double robotY = (PlayField.getSideLength()/2 - RobotSize.xSize.getSizeValue()/2);
            return robotY;
        }

    }

    /*****************************************************************
    //******    CONSTRUCTORS
    //****************************************************************/

    public Pose(){
        //this.x = 0.0;
        //this.y = 0.0;
        this.fieldPosition = new FieldPosition();
        this.headingDeg = 0.0;
        this.newHeadingReadingDeg = headingDeg;
    }     //Default constructor

    public Pose(double xInput, double yInput, double headingInputDeg){
        this.fieldPosition = new FieldPosition(xInput,yInput);
        //this.x = xInput;
        //this.y = yInput;
        this.headingDeg = UtilFuncs.applyAngleBounds(headingInputDeg);
        this.newHeadingReadingDeg = headingDeg;

    }

    public Pose(FieldPosition fp, double headingInputDeg){
        this.fieldPosition = fp;
        this.headingDeg = UtilFuncs.applyAngleBounds(headingInputDeg);
        this.newHeadingReadingDeg = headingDeg;

    }

    public Pose(Alliance alliance, StartingSide startingSide){
        //Start on the bottom wall, heading = 0
        double robotHalfWidth = RobotSize.ySize.getSizeValue()/2;
        // Note that robot sees LEFT and MIDDLE barcode when RED
        //  and MIDDLE and RIGHT when BLUE
        int numTiles = (startingSide == StartingSide.CAROUSEL) ? 2 : 0;
        double xCenter = (-FieldTile.getSize() * numTiles) + robotHalfWidth;
        if(AllianceSingleton.isBlue()){
            // add 4 inches for alignment
            xCenter += xCenter;
        }

        double yCenter = (PlayField.getSideLength()/2 - RobotSize.xSize.getSizeValue()/2);  //assumes blue
        int allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;
        yCenter *= allianceSign;    //flips sign for RED

        double startingHeadingDeg = -90;        // assumes blue
        startingHeadingDeg *= allianceSign;     // flips heading for RED

        this.fieldPosition = new FieldPosition(xCenter, yCenter);
        this.headingDeg = startingHeadingDeg;
        this.newHeadingReadingDeg = startingHeadingDeg;
    }

    //  When using a pre-defined StartingPose from the enumeration
    @Deprecated
    public Pose(PresetPose presetPose, Alliance alliance) {
        this(presetPose.getXStart(), presetPose.getYStart(), presetPose.getHeadingStart());

        //Now flip the sign of the y component if on the red alliance
        if(alliance == Alliance.RED){
            this.fieldPosition.setyPosition(-this.fieldPosition.getyPosition());
        }
    }

    // When using a StartLine.LinePosition
    @Deprecated
    public Pose(StartLine.LinePosition linePosition, Alliance alliance){
        double xPosition = -new PlayField().getFieldYSize()/2 + RobotSize.xSize.getSizeValue()/2;
        //Assumed blue alliance, robot heading 0, right wheels on start line
        double yPosition = linePosition.getyCenter() + RobotSize.ySize.getSizeValue()/2;
        this.fieldPosition = new FieldPosition(xPosition, yPosition);
        this.headingDeg = 0;
        this.newHeadingReadingDeg = headingDeg;

        //Now flip the sign of the y component if on the red alliance
        if(alliance == Alliance.RED){
            this.fieldPosition.setyPosition(-this.fieldPosition.getyPosition());
        }

    }


    /*****************************************************************
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/
    public double getX() { return this.fieldPosition.getPositionComponent(CsysDirection.X);}
    public double getY() { return this.fieldPosition.getPositionComponent(CsysDirection.Y); }
    public double getZ() { return this.fieldPosition.getPositionComponent(CsysDirection.Z); }

    public FieldPosition getFieldPosition(){return this.fieldPosition;}
    public double getCoordinate(CsysDirection dir){
        double coordinateValue = 0;
        if(dir == CsysDirection.Heading){
            coordinateValue = headingDeg;
        } else{
            coordinateValue = this.fieldPosition.getPositionComponent(dir);
        }
        return coordinateValue;
    }

    public double getHeadingDeg() { return headingDeg;}
    public double getHeadingRad(){ return Math.toRadians(headingDeg); }
    public double getNewHeadingReadingDeg(){return this.newHeadingReadingDeg;}
    public double getNewHeadingReadingRad(){return Math.toRadians(this.newHeadingReadingDeg);}

    public void setX(double x) {
        this.fieldPosition.setxPosition(x);
    }

    public void setY(double y) {
        this.fieldPosition.setyPosition(y);
    }

    public void setFieldPosition(FieldPosition fieldPosition) {
        this.fieldPosition = fieldPosition;
    }

    public void setHeadingDeg(double headingDeg) {
        /** Sets heading, but makes sure it is within the legal bounds
         *  which is -180 < heading <= 180
         */
        headingDeg = UtilFuncs.applyAngleBounds(headingDeg);
        this.headingDeg = headingDeg;
    }

    public void setNewHeadingReadingDeg(double headingReadingDeg){
        headingReadingDeg = UtilFuncs.applyAngleBounds(headingReadingDeg);
        this.newHeadingReadingDeg = headingReadingDeg;
    }

    public void updateTo(Pose pose){
        this.fieldPosition = pose.getFieldPosition();
        this.headingDeg = pose.getHeadingDeg();
    }


    /***************************************************************88
    //******    Instance Methods
    //***************************************************************/

    public void updateHeadingWithReading(){
        this.headingDeg = this.newHeadingReadingDeg;
    }

    @Override
    public String toString(){
        StringBuilder sb = new StringBuilder();

        //Loop through the coordinates
        boolean firstPass = true;
        for(CsysDirection dir: CsysDirection.values()){
            if(firstPass) sb.append("Pose: (");
            else if (dir == CsysDirection.Heading) sb.append(") @ ");
            else sb.append(", ");

            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f",this.getCoordinate(dir));
            if (dir == CsysDirection.Heading) sb.append("°");
            firstPass = false;
        }

        return sb.toString();

        //return "(" + String.format("%.2f",fieldPosition.getxPosition()) + " ," + String.format("%.2f",fieldPosition.getyPosition()) + " @ "
        //        + String.format("%.2f", headingDeg) + ")";
    }

    /***************************************************************88
     //******    Class METHODS
     //***************************************************************/

}
