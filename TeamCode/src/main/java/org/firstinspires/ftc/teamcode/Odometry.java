package org.firstinspires.ftc.teamcode;

public class Odometry {
    private static final double wheelRadius = 4.0;
    private static final int ticksPerRev = 8120;
    private static final double gearRatio = 1;

    private static final double L = 4.0;
    private static final double F = 4.0;

    //we will need to tune these values
    private static final double xMultiplier=1;
    private static final double yMultiplier=1;

    private double x=0;
    private double y=0;
    private double angle=0;
    private double previousLeftTicks=0;
    private double previousRightTicks=0;
    private double previousFrontTicks=0;
    public void updateOdometry(int leftTicks, int rightTicks, int frontTicks){
        double leftPos = ticksToInches(leftTicks);
        double rightPos = ticksToInches(rightTicks);
        double frontPos = ticksToInches(frontTicks);

        double changeLeft = leftPos-previousLeftTicks;
        double changeRight = rightPos-previousRightTicks;
        double changeFront = frontPos-previousFrontTicks;

        previousLeftTicks = leftPos;
        previousRightTicks = rightPos;
        previousFrontTicks = frontPos;

        double changeAngle = (changeLeft-changeRight)/L;

        double changeVertical=  (changeLeft + changeRight) /2;
        double changeHorizontal = changeFront-(F*changeAngle);

        double changeX = changeVertical*Math.cos(changeAngle)-changeHorizontal*Math.sin(changeAngle);
        double changeY =changeVertical*Math.sin(changeAngle)+changeHorizontal*Math.cos(changeAngle);

        x+=changeX*xMultiplier;
        y+=changeY*yMultiplier;
        angle+=changeAngle;
        normalizeAngle();
    }

    public double getX() {return x;}
    public double getY() {return y;}
    public double getAngle() {return Math.toDegrees(angle);}

    private void normalizeAngle() {
        angle %=2*Math.PI;
    }

    private double ticksToInches(int ticks) {
        double circumference = 2*Math.PI * wheelRadius;
        return ((double) ticks /ticksPerRev)*circumference*gearRatio;
    }

}
