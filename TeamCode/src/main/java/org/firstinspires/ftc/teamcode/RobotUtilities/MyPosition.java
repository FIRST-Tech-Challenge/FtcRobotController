package org.firstinspires.ftc.teamcode.RobotUtilities;

//import HelperClasses.Robot;

public class MyPosition {

//    public static Robot myRobot;
    // Gluten Free Values.
//    public static double moveScalingFactor = 12.56064392;
//    public static double auxScalingFactor = 12.48;//12.6148;
//    public static double turnScalingFactor = 35.694;

    //    public static double moveScalingFactor = 4.1146937;
    // Make bigger to make distance shorter
//    public static double moveScalingFactor = 4.15584064; STEM Punk
//    public static double moveScalingFactor = 1.461037725; Ultimate Goal Math Starting Point
    public static double moveScalingFactor = 1.461037725;

//    public static double auxScalingFactor = 4.08827586;
// Make bigger to make distance shorter
//    public static double auxScalingFactor = 4.12915862; STEM Punk
//    public static double auxScalingFactor = 1.451657327; Ultimate Goal Math Starting Point
    public static double auxScalingFactor = 1.451657327;
//    public static double turnScalingFactor = 39.0;
//    public static double turnScalingFactor = 11.6928621;
//    public static double turnScalingFactor = 10.2897186;
    // 10.49 goes less than 0 cw
    // 10.4999 goes than 0 cw
    // 10.5 goes more than 0 cw
    // 10.525 goes more than 0 cw
    // 10.7 goes more than 0
//    public static double turnScalingFactor = 10.4999; STEM Punk
//    public static double turnScalingFactor = 3.691371094; Ultimate Goal Math Starting Point
    public static double turnScalingFactor = 3.691371094;
    public static double auxPredictionScalingFactor = 0.92;


    public static double wheelLeftLast = 0.0;
    public static double wheelRightLast = 0.0;
    public static double wheelAuxLast = 0.0;

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double worldXPositionOld = 0.0;
    public static double worldYPositionOld = 0.0;


    public static double currPos_l = 0;
    public static double currPos_r = 0;
    public static double currPos_a = 0;
    


    //stuff for reading the angle in an absolute manner
    public static double wheelLeftInitialReading = 0.0;
    public static double wheelRightInitialReading = 0.0;
    public static double lastResetAngle = 0.0;//this is set when you reset the position


    //use this to get how far we have traveled in the y dimension this update
    public static double currentTravelYDistance = 0.0;


    public static void initialize(double l, double r,double a){
        currPos_l = l;
        currPos_r = r;
        currPos_a = a;
        update();
    }

    // myRobot was passed in just for telemetry printing.
    //    public static void initialize(double l, double r,double a, Robot myRobot){
//        MyPosition.myRobot = myRobot;
//        currPos_l = l;
//        currPos_r = r;
//        currPos_a = a;
//        update();
//    }

    public static void giveMePositions(double l, double r, double a){
//        if(currPos_l != 0 || currPos_r != 0 || currPos_a != 0) {
            currPos_l = l;
            currPos_r = r;
            currPos_a = a;
            update();
//        }
    }

    private static void update(){
        PositioningCalculations();
    }

    public static float AngleWrap(float angle){
		return (float)AngleWrap((double)angle);
    }

    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0* Math.PI;
        }
        while (angle> Math.PI){
            angle -= 2.0* Math.PI;
        }
        return angle;
    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PositioningCalculations(){
        // This was due to GF encoder must have been reverse.
        double wheelLeftCurrent = -currPos_l;
//        double wheelLeftCurrent = currPos_l;
        double wheelRightCurrent= currPos_r;
        double wheelAuxCurrent = currPos_a;

        //compute how much the wheel data has changed
        double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


        //get the real distance traveled using the movement scaling factors
        double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
        double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
        double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;

        //get how much our angle has changed
        double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;
//        myRobot.telemetry.addLine("Angle increment is " +
//                (angleIncrement > 0 ? "POSITIVE" : "NEGATIVE"));


        //but use absolute for our actual angle
        double wheelRightTotal = currPos_r-wheelRightInitialReading;
        // This was due to GF encoder must have been reverse. or something else?
        double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);
//        double wheelLeftTotal = currPos_l-wheelLeftInitialReading;

        double worldAngleLast = worldAngle_rad;
        worldAngle_rad = -AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);

        //get the predicted amount the straif will go
        double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
        //now subtract that from the actual
        double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


        //relativeY will by defa
        double relativeY = (wheelLeftDeltaScale + wheelRightDeltaScale)/2.0;
        double relativeX = r_xDistance;

//        myRobot.telemetry.addLine("left wheel: " + (wheelLeftCurrent*moveScalingFactor/1000.0));
//        myRobot.telemetry.addLine("right wheel: " + (wheelRightCurrent*moveScalingFactor/1000.0));
//        myRobot.telemetry.addLine("aux wheel: " + (wheelAuxCurrent*auxScalingFactor/1000.0));


        //if angleIncrement is > 0 we can use steven's dumb stupid and stupid well you know the point
        //equations because he is dumb
        if(Math.abs(angleIncrement) > 0){
            //gets the radius of the turn we are in
            double radiusOfMovement = (wheelRightDeltaScale+wheelLeftDeltaScale)/(2*angleIncrement);
            //get the radius of our straifing circle
            double radiusOfStraif = r_xDistance/angleIncrement;





            relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1 - Math.cos(angleIncrement)));

            relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

//            myRobot.telemetry.addLine("radius of movement: " + radiusOfMovement);
//            myRobot.telemetry.addLine("radius of straif: " + radiusOfStraif);
//            myRobot.telemetry.addLine("relative y: " + relativeY);
//            myRobot.telemetry.addLine("relative x: " + relativeX);
        }



        worldXPosition += (Math.cos(worldAngleLast) * relativeY) + (Math.sin(worldAngleLast) *
                relativeX);
        worldYPosition += (Math.sin(worldAngleLast) * relativeY) - (Math.cos(worldAngleLast) *
                relativeX);


        SpeedOmeter.yDistTraveled += relativeY;
        SpeedOmeter.xDistTraveled += r_xDistance;



        //save the last positions for later
        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        //save how far we traveled in the y dimension this update for anyone that needs it
        //currently the absolute control of the collector radius uses it to compensate for
        //robot movement
        currentTravelYDistance = relativeY;
    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PositioningCalculationsOld(){
        double wheelLeftCurrent = currPos_l;
        double wheelRightCurrent= -currPos_r;
        double wheelAuxCurrent = currPos_a;

        //compute how much the wheel data has changed
        double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


        //get the real distance traveled using the movement scaling factors
        double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
        double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
        double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;



        //get how much our angle has changed
        double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;



        //but use absolute for our actual angle
        double wheelRightTotal = currPos_r-wheelRightInitialReading;
        double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);
        worldAngle_rad = AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);




        //relative y translation
        double r_yDistance = (wheelRightDeltaScale+wheelLeftDeltaScale)/2;


        double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
        double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


        worldXPosition += (Math.cos(worldAngle_rad) * r_yDistance) + (Math.sin(worldAngle_rad) *
                r_xDistance);
        worldYPosition += (Math.sin(worldAngle_rad) * r_yDistance) - (Math.cos(worldAngle_rad) *
                r_xDistance);



        SpeedOmeter.yDistTraveled += r_yDistance;
        SpeedOmeter.xDistTraveled += r_xDistance;



        //save the last positions for later
        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        //save how far we traveled in the y dimension this update for anyone that needs it
        //currently the absolute control of the collector radius uses it to compensate for
        //robot movement
        currentTravelYDistance = r_yDistance;

    }
    public static double subtractAngles(double angle1, double angle2){
        return AngleWrap(angle1-angle2);
    }




    /**USE THIS TO SET OUR POSITION**/
    public static void setPosition(double x,double y,double angle){
        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad= angle;

        worldXPositionOld = x;
        worldYPositionOld = y;

        //remember where we were at the time of the reset
        wheelLeftInitialReading = currPos_l;
        wheelRightInitialReading = currPos_r;
        lastResetAngle = angle;
    }
}
