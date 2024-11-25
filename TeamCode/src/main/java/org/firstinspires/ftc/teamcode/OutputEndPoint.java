package org.firstinspires.ftc.teamcode;


//this recives the 2d point jand wrist pitch, roll, and is gripper open
//from, there, we calculate the compenont positions at init
//we have those vales saved, so we don't have to do the math each frame, instead only once at the start
public class OutputEndPoint {
    //input values
    public Point2d wristPoint;
    public double wristPitchRelativeToGroundDegrees, roll;
    public boolean open;

    //output values
    public double slideInches;
    public double armDegrees;

    //constants
    static final double WRIST_LENGTH = 3.268;
    static final double ARM_LENGTH = 13.3;
    static final double SLIDE_ANGLE_DEG = 100;
    static final double SLIDE_SLOPE = Math.sin(Math.toRadians(SLIDE_ANGLE_DEG)) / Math.cos(Math.toRadians(SLIDE_ANGLE_DEG));
    static final double SLIDE_MAX_EXTENSION = 18.89764;
    static final Point2d SLIDE_START_POINT = new Point2d(-5.56, 15.171);
    static final Point2d SLIDE_MIN_POINT = new Point2d(
            SLIDE_START_POINT.x + SLIDE_MAX_EXTENSION*Math.cos(Math.toRadians(SLIDE_ANGLE_DEG)),
            SLIDE_START_POINT.y + SLIDE_MAX_EXTENSION*Math.sin(Math.toRadians(SLIDE_ANGLE_DEG))
    );
    static final double SLIDE_Y_INTERCEPT = SLIDE_START_POINT.y - SLIDE_SLOPE*SLIDE_START_POINT.x;

    Point2d armPoint;
    Point2d slidePoint = new Point2d(0,0);

    //tracking point of triangle on end of arm, should about the center of the sample

    //horizontal values [-17, 12]ish
    //vertical values [5.5, 48]ish
    //pitch values [0, 360] (this is weird because it depends on where the arm is) (i don't think this is quite right yet)
    //roll values [0, 90]
    //open values [true, false]
    //THIS CONSTRUCTOR RECEIVES THE POINTS AND CALCULATES THE HARDWARE POSITIONS
    public OutputEndPoint(Point2d wristPoint,
                           double wristPitchRelativeToGroundDegrees, double roll,
                           boolean open) {
        this.wristPoint = wristPoint;
        this.wristPitchRelativeToGroundDegrees = wristPitchRelativeToGroundDegrees;
        this.roll =  roll;
        this.open =  open;
        calculateComponentPositions();
    }
    //THIS CONSTRUCTOR RODES THE OPPOSITE. it RECEIVES THE HARDWARE POSITIONS AND FINDS THE POINTS
    public OutputEndPoint(double slideInches,
                          double armDegrees,
                          double wristPitchRelativeToGroundDegrees, double roll,
                          boolean open) {
        this.slideInches = slideInches;
        this.armDegrees = armDegrees;
        this.wristPitchRelativeToGroundDegrees = wristPitchRelativeToGroundDegrees;
        this.roll = roll;
        this.open = open;

        calculatePoints();
    }

    //normal kinematics
    //find the points based of off hardware positions
    public void calculatePoints() {
        double slideX = SLIDE_START_POINT.x+Math.cos(Math.toRadians(SLIDE_ANGLE_DEG))*slideInches;
        slidePoint = pointOnSLideFromX(slideX);
        armPoint = new Point2d(slidePoint.x + Math.cos(Math.toRadians(armDegrees))*ARM_LENGTH,
                               slidePoint.y + Math.sin(Math.toRadians(armDegrees))*ARM_LENGTH
        );
        wristPoint = new Point2d(armPoint.x + Math.cos(Math.toRadians(wristPitchRelativeToGroundDegrees))*WRIST_LENGTH,
                                 armPoint.y + Math.sin(Math.toRadians(wristPitchRelativeToGroundDegrees))*WRIST_LENGTH
        );
    }

    //inverse kinematics
    public void calculateComponentPositions() {
        boolean success = calculateArmAndSlidePoints();
        if (success) {
            slideInches = calculateSlideDistance();
            //verticalSlides.setTargetInches(slideDistanceInches);
            armDegrees = calculateArmAngle();
        }
        //gripper.setPosition(outputEndPointTarget.open);
    }//club test

    public double calculateSlideDistance() {
        return slidePoint.distanceToOtherPoint(SLIDE_START_POINT);
    }
    public double calculateArmAngle() {
        return slidePoint.angleBetweenPoints(armPoint);
    }

    public boolean calculateArmAndSlidePoints() {
        calculateArmPoint();
        return calculateSlidePoint();
    }


    public boolean calculateSlidePoint() {
        double a = Math.pow(SLIDE_SLOPE, 2) + 1;
        double b = -2 * armPoint.x + 2 * SLIDE_SLOPE * (SLIDE_Y_INTERCEPT - armPoint.y);
        double c = Math.pow(armPoint.x, 2) + Math.pow(SLIDE_Y_INTERCEPT, 2) - 2 * SLIDE_Y_INTERCEPT * armPoint.y + Math.pow(armPoint.y, 2) - Math.pow(ARM_LENGTH, 2);
        try {
            double lowPoint = (-b + Math.sqrt(Math.pow(b, 2) - 4*a*c)) / (2 * a);
            if (lowPoint <= SLIDE_START_POINT.x && lowPoint >= SLIDE_MIN_POINT.x) {
                slidePoint = pointOnSLideFromX(lowPoint);
            } else {
                double highPoint = (-b - Math.sqrt(Math.pow(b, 2) - 4*a*c)) / (2 * a);
                if (highPoint <= SLIDE_START_POINT.x && highPoint >= SLIDE_MIN_POINT.x) {
                    slidePoint = pointOnSLideFromX(highPoint);
                }
            }
        } catch (Exception exception) {
            return false;
        }
        return true;
    }

    public void calculateArmPoint() {
        armPoint = new Point2d(
                wristPoint.x + (WRIST_LENGTH*Math.cos(Math.toRadians(wristPitchRelativeToGroundDegrees + 180))),
                wristPoint.y + (WRIST_LENGTH*Math.sin(Math.toRadians(wristPitchRelativeToGroundDegrees + 180)))
        );
    }

    public Point2d pointOnSLideFromX(double x) {
        return  new Point2d(
                x,
                SLIDE_SLOPE*x + SLIDE_Y_INTERCEPT
        );
    }



    public String pointTelemetry() {
        return "\nWrist: " + wristPoint.toString() + "\nArm: " + armPoint.toString() + "\nSlide: " + slidePoint.toString();
    }

    public String componentValuesIrl() {
        return "Wrist: " + (wristPitchRelativeToGroundDegrees + armDegrees) + "\nArm: " + armDegrees + "\nSlide: " + slideInches + "\nGripper Open: " + open;
    }
}
