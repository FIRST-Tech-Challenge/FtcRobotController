package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class Navigation {
    public enum rotationDirection {CLOCKWISE, COUNTERCLOCKWISE};
    public static enum Action {NONE, SLIDES_LOW, SLIDES_HIGH,}; //Makes actions of the Robot that can be used anywhere within the folder.

    //**AUTONOMOUS CONSTANTS**
    public enum MovementMode {FORWARD_ONLY, BACKWARD_ONLY, STRAFE_LEFT, STRAFE_RIGHT}; //Movements within the robot Autonomous Mode
    public final double STRAFE_ACCELERATION = 0.1; //Number indicated Inches per second squared
    public final double ROTATE_ACCELERATION = 0.1; //Number indicated Radians per second squared
    public final double SPEED_FACTOR = 0.7; //Speed of the robot when all motors are set to full power
    static final double STRAFE_RAMP_DISTANCE = 4; //Number indicated Inches
    //static final double ROTATION_RAMP_DISTANCE = Math.PI / 2; //NOT BEING USED
    static final double MIN_STRAFE_POWER = 0.3; //Sets the strafe power to 3/10th power
    static final double MAX_STRAFE_POWER = 0.5; //Sets the strafe power to 5/10th power
    static final double STRAFE_CORRECITON_POWER = 0.3; //idk what this means
    static final double STRAFE_SLOW = 0.1; //idk what this means
    static final double MAX_ROTATION_POWER = 0.3; //sets the rotation power to 3/10th, power
    static final double MIN_ROTATION_POWER = 0.03; //sets the rotation power to 3/100th power (why?)
    static final double ROTATION_CORRECTION_POWER = 0.04; //idk what this means

    // Accepted amounts of deviation between the robot's desired position and actual position.
    static final double EPSILON_ANGLE = 0.35;
    //   static final int NUM_CHECK_FRAMES = 5; //The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.

    //Distances between where the robot extends/retracts the linear slides and where it opens the claw.
    static final double ROTATION_TIME = 1050; //???
    static final double FLOAT_EPSILON = 0.001; //????


    //**TELEOP CONSTANTS**
    static final double MOVEMENT_MAX_POWER = 1; //Sets the maximum power to full power. (Full power is between 0 - 1)
    static final double ROTATION_POWER = 0.5; //Sets the maximum rotation power 1/2 full power
    static final double REDUCED_ROTATION_POWER = 0.2; //Lets the minimum rotation power to 1/5th full power
    static final double SLOW_MOVEMENT_SCALE_FACTOR = 0.3; //idk what this means
    static final double MEDIUM_MOVEMENT_SCALE_FACTOR = 0.6; //idk what this means


    //**INSTANCE ATTRIBUTES**//
    public double[] wheel_speeds = {0.95, 1, -1, -0.97}; //Back left, Back right, Front left, Front right. Temporary Note: currently FR from -0.90 to -0.92
    public double strafePower; //This is for Tele-Op ONLY.

    /*
     First position in this ArrayList is the first position that robot is planning to go to.
     This condition must be maintained (positions should be deleted as the robot travels)
     NOTE: a position is both a location and a rotation.
     NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
     */
    public ArrayList<Position> path; //List of positions that the robot will go into WHEN IT IS IN AUTOMOTOUS MODE.
    public int pathIndex; //Index of the path array list.

    /**
     * @param path positions of where the robot is traveling to in auton
     * @param allianceColor alliance color on what team we are on (which is either red or blue)
     * @param startingSide the starting side on where our robot is starting from (on the field)
     * @param movementMode the movement within the robot
     */
    public Navigation(ArrayList<Position> path, RobotManager.AllianceColor allianceColor, RobotManager.StartingSide startingSide, MovementMode movementMode){
        this.path = path;
        this.movementMode = movementMode;
        pathIndex = 0;
    }

    /**
     * @param startingSide where the robot starts in auton mode on the field
     * @param parkingPosition the parking position of the robot during auton mode.
     */
    public void configurePath(RobotManager.StartingSide startingSide, RobotManager.ParkingPosition parkingPosition){
        transformPath(startingSide);
        //Set parking location
        setParkingLocation(startingSide, parkingPosition);
    }

    /** Makes the robot travel along the pth until it reaches a POI (Position of Interest)
     * @param robotManager the robot manager of the robot
     * @param robot the physical robot itself
     */
    public Position travelToNextPOI(RobotManager robotManager, Robot robot) {
        if (path.size() <= pathIndex) {
            robot.telemetry.addData("Path size <= to the path index, end of travel. pathIndex:",pathIndex); //This will show on the console. (Phone)
            return null;
        }
        Position target = path.get(pathIndex);
        robot.positionManager.updatePosition(robot); //This constantly updates the position on the robot on the field.
        robot.telemetry.addData("Going to", target.getX() + ", " + target.getY()); //Updating the X and Y value to the driver station (AKA: the phone)
        robot.telemetry.addData("name", target.getName()); //Gets the name.

        switch(movementMode){
            case FORWARD_ONLY: //Robot moving forward ONLY (if equal with movementMode)
                rotate(getAngleBetween(robot.getPosition(), target) - Math.PI / 2, target.rotatePower, robot);
                travelLinear(target, target.getStrafePower(), robot);
                rotate(target.getRotation(), target.getRotatePower(), robot);
                break; //case statement ends.

            case STRAFE://go directirly to the target. do not care about the direction the robot is facing during travle
                travelLinear(target, target.strafePower, robot);
                double difference;
                if(pathIndex > 0){ //
                    difference = target.getRotation()-path.get(pathIndex-1).getRotation();
                } else { //whatever the current rotation is...
                    difference = target.getRotation();
                }
                robot.telemetry.addData("Difference", difference);
                robot.telemetry.addData("Target", target);
                robot.telemetry.update();
                //deadReckoningRotation(robotManager, robot, difference, target.rotatePower);
                break;

            case BACKWARD_ONLY: //Robot moving backward ONLY (if equal with movementMode)
                rotate(getAngleBetween(robot.getPosition(), target) - Math.PI*3 / 2, target.rotatePower, robot);
                travelLinaer(target, target.getStrafePower(), robot);
                rotate(target.getRotation(), target.getRotatePower(), robot);
                break;
        }
        pathIndex++; //increments path index to the next value...
        robot.telemetry.addData("Got to", target.name); //debug thingy for auton (since there were fun times with it...)
        return path.get(pathIndex - 1); //Return the point where the robot is currently at
    }


}
//Coders: Tyler M.
//Alumni Help: Stephen D.