package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderSetup;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotOrientation;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;

import java.util.ArrayList;

/**
 *   CLASS:     PoseChange
 *   INTENT:    works with EbotsMotionController to quantify changes between loop iterations
 *              Performs calculations to discern encoder values with respect to spin and translation
 */
public class PoseChange {

    private double spinAngleDeg;    //This contains the angle that occurred during a loop
    private FieldPosition incrementalRobotMovement;
    private FieldPosition incrementalFieldMovement;

    private final String logTag = "EBOTS";
    private final boolean debugOn = false;

    public PoseChange(EbotsRobot2020 robot) {
        if(debugOn) Log.d(logTag, "Entering PoseChange (Robot) constructor...");
        //Start by calculating the spin angle
        this.spinAngleDeg = calculateSpinAngle(robot);
        this.calculateRobotMovement(robot);     //Calculates in both robot and field csys,
                                                //stores in class variables incremental[csys]Movement
    }

    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/
    public double getSpinAngleDeg() {
        return spinAngleDeg;
    }
    public FieldPosition getIncrementalRobotMovement() {return incrementalRobotMovement;}
    public FieldPosition getIncrementalFieldMovement() {return incrementalFieldMovement;}

    /***************************************************************
     //******    Class METHODS
     //***************************************************************/

    private double calculateSpinAngle(EbotsRobot2020 robot) {
        //Spin clicks are calculated differently between 2-wheel and 3-wheel strategy
        //  For 2 wheel, must rely on the imu to estimate spin angle
        //  For 3 wheel, look at the difference between the encoders in the encoderDoubleDirection
        double spinAngleDeg = 0;

        if (robot.getEncoderSetup() == EncoderSetup.TWO_WHEELS) {
            double currentHeadingDeg = robot.getActualPose().getHeadingDeg();
            double newHeadingReadingDeg = robot.getActualPose().getNewHeadingReadingDeg();
            double changeInHeadingDeg = newHeadingReadingDeg - currentHeadingDeg;
            spinAngleDeg = Pose2020.applyAngleBound(changeInHeadingDeg);

        } else if (robot.getEncoderSetup() == EncoderSetup.THREE_WHEELS) {
            //Loop through the encoders and calculate the spin clicks for the double direction
            double spinClicks = 0;
            double spinDistance = 0;
            double spinRadius = 0;
            for (EncoderTracker e : robot.getEncoderTrackers()) {
                if (e.getRobotOrientation() == robot.getEncoderSetup().getDoubleEncoderDirection()) {
                    //Now, in order to get the difference between the 2 encoders,
                    //   assign a sign based on robot orientation of the encoder
                    //   Since spinning CCW is positive, wheels on the right have increasing click counts with angle
                    //   Example:  Right wheel sees +200 and gets +1 sign applied (SpinBehavior.INCREASES_WITH_ANGLE)
                    //             Left wheel sees +100 and gets -1 sign applied (SpinBehavior.DECREASES_WITH_ANGLE)
                    //             Spin clicks for the robot equal 200 + (-100) = 100
                    int appliedSign = getAppliedSign(e);
                    spinClicks = (appliedSign * e.getIncrementalClicks());
                    //  Convert the spin clicks into distance traveled, this is used to get the difference
                    //  between the 2 sides
                    spinDistance += (spinClicks / e.getClicksPerInch());
                    //  The two spin radii will be averaged to determine the angle
                    //  These two radii should be equal, but in case they aren't, the avg is used
                    spinRadius += e.getSpinRadius();
                }
            }
            //Get the average spin radius (in case they are different between the 2 encoders)
            spinRadius /= 2.0;
            //Calculate an "effective" spin radius
            double spinCircumference = 2.0 * Math.PI * spinRadius;
            //Calculate the angle
            spinAngleDeg = (spinDistance / spinCircumference) * 360.0;

            //Calculate and set the new heading reading by applying increment to robot's current heading
            double newHeadingReadingDeg = robot.getActualPose().getHeadingDeg() + spinAngleDeg;
            robot.getActualPose().setNewHeadingReadingDeg(newHeadingReadingDeg);
        }
        return spinAngleDeg;
    }

    private void calculateRobotMovement(EbotsRobot2020 robot) {
        if(debugOn) Log.d(logTag, "Entering PoseChange.calculateRobotMovement...");
        //  Based on the encoder setup, determine the divisor of the component
        //  In 3-wheel systems, encoders aligned in doubleEncoderDirection are averaged
        //  Since double encoder clicks are being averaged, must divide each translationClicks by 2
        //  This generates 2 entries for the 2-wheel system and 3 entries for the 3-wheel system
        //  Entries are stored in the array movementComponents class variable
        //  Then they are summed and written to incrementalRobotMovement
        //  Then transformed into field coordinates and written to incrementalFieldMovement

        RobotOrientation doubleEncoderDirection = robot.getEncoderSetup().getDoubleEncoderDirection();
        ArrayList<MovementComponent> movementComponents = new ArrayList<>();    //this array contains movement component objects
        if(movementComponents.size()>0) movementComponents.clear(); //Make sure it is empty
        //It contains MovementComponents for both field and robot coordinate systems
        //And it contains them for both X and Y CSys Directions
        //Access the total net movement using the getMovementComponentInches method

        //Now loop through the encoders
        for (EncoderTracker e: robot.getEncoderTrackers()) {
            //Set the divisor to 2 if encoder is oriented in double direction
            RobotOrientation encoderRobotOrientation = e.getRobotOrientation();
            double clickDivisor = (encoderRobotOrientation == doubleEncoderDirection) ? 2 : 1;
            //Set the coordinate system direction based on encoder orientation
            CsysDirection csysDirection = (encoderRobotOrientation == RobotOrientation.FORWARD) ? CsysDirection.X : CsysDirection.Y;

            //Get the translation clicks and halve if required
            double translationClicks = (calculateSingleEncoderTranslationClicks(e) / clickDivisor);

            //Convert clicks to a distance based on encoder sensitivity and size
            double distanceComponent = translationClicks / e.getClicksPerInch();

            //Create a Movement Coordinate object and store in the array
            MovementComponent movement = new MovementComponent(CoordinateSystem.ROBOT, csysDirection, distanceComponent);
            movementComponents.add(movement);
        }

        double xTranslationRobotCsys = getMovementComponentInches(movementComponents, CoordinateSystem.ROBOT, CsysDirection.X);
        double yTranslationRobotCsys = getMovementComponentInches(movementComponents, CoordinateSystem.ROBOT, CsysDirection.Y);
        incrementalRobotMovement = new FieldPosition(xTranslationRobotCsys,yTranslationRobotCsys,CoordinateSystem.ROBOT);
        incrementalFieldMovement = CoordinateSystem.transformCoordinateSystem(incrementalRobotMovement,CoordinateSystem.FIELD,robot.getActualPose().getHeadingDeg());
        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("incrementalRobotMovement: ");
            sb.append(incrementalRobotMovement.toString());
            sb.append(" incrementalFieldMovement: ");
            sb.append(incrementalFieldMovement.toString());
            Log.d(logTag, sb.toString());
        }
    }

        private int calculateSingleEncoderTranslationClicks(EncoderTracker encoderTracker){
        // Each wheel incurs clicks from both translation and rotation
        //      new_clicks = translation_clicks + rotation_clicks
        //      or translation_clicks = new_clicks - rotation_clicks
        if (debugOn) Log.d(logTag, "Entering PoseChage.calculateSingleEncoderTranslationClicks...");

        int spinClicks = (int) Math.round(Math.toRadians(this.spinAngleDeg) * encoderTracker.getSpinRadius() * encoderTracker.getClicksPerInch());
        //the sign of spinClicks should align with the angle direction (CCW is positive)
        int appliedSign = getAppliedSign(encoderTracker);
        int translationClicks = encoderTracker.getIncrementalClicks() - (appliedSign * spinClicks);
        if(debugOn) {
            StringBuilder sb = new StringBuilder();
            sb.append(encoderTracker.toString() + "\n");
            sb.append("Spin Clicks: ");
            sb.append(spinClicks);
            sb.append(" Incremental encoder clicks: ");
            sb.append(encoderTracker.getIncrementalClicks());
            sb.append(" Net translation clicks: ");
            sb.append(translationClicks);
            Log.d(logTag, sb.toString());
        }
        return translationClicks;
    }

    public double getMovementComponentInches(ArrayList<MovementComponent> movementComponents, CoordinateSystem csys, CsysDirection dir){
        double totalMovement = 0.0;
        for(MovementComponent mc: movementComponents){
            if(mc.getCoordinateSystem() == csys && mc.getCsysDirection() == dir){
                totalMovement += mc.getDistanceInches();
            }
        }
        return totalMovement;
    }


    private int getAppliedSign(EncoderTracker encoderTracker) {
        int appliedSign = (encoderTracker.getSpinBehavior() == EncoderTracker.SpinBehavior.INCREASES_WITH_ANGLE) ? 1 : -1;
        return appliedSign;
    }

    public String toString(EbotsRobot2020 robot){
        String returnString;
        returnString = "Movement in robot reference frame at heading: "
                + "@" + String.format("%.2f",robot.getActualPose().getHeadingDeg())
                + " (" + String.format("%.2f",this.incrementalRobotMovement.getPositionComponent(CsysDirection.X))
                + " , " + String.format("%.2f",this.incrementalRobotMovement.getPositionComponent(CsysDirection.Y))
                + ") \n" + "Movement in the field reference frame: "
                + " (" + String.format("%.2f",this.incrementalFieldMovement.getPositionComponent(CsysDirection.X))
                + " , " + String.format("%.2f",this.incrementalFieldMovement.getPositionComponent(CsysDirection.Y))
                + ")";
        return returnString;

    }
}
