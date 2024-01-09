//package org.firstinspires.ftc.teamcode.AutoCode.Control;
//
//import com.qualcomm.robotcore.util.MovingStatistics;
//
//import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
//import org.firstinspires.ftc.teamcode.Vector;
//
//
//public class TrackingWheelIntegrator
//{
//    private double x, y;
//
//    private double oldX, oldY;
//
//    private int lastPosLeft, lastPosRight, lastPosAux;
//
//    private int dPosLeft, dPosRight, dPosAux;
//
//    double dH;
//
//    int cPosLeft;
//    int cPosRight;
//    int cPosAux;
//
////GLOBALS
//    private double dX, dY, heading;
//    Vector trackingVector = new Vector(); //296.90372 //288.17579 //288.6475478 //288.9193056 worked //290.67
//    Vector instantVector = new Vector(); //-166.7675; before spring change.
//    private double TICKS_PER_DEGREE = -285.3; //(-401.6311) 24 inch BASE. (-288.17579) Undershot 19 degrees off for 10 spins 13x15 BASE (-288.9193056) Under shoot) //290 perfect 13x15
//    private double TICKS_PER_INCH_NORMAL = -1710; //1713.75 //1730.85
//    private double TICKS_PER_INCH_STRAFE = 1710; //1702.84
//    double AUX_SPIN_SLIP = 0.101919839; // 0.101919839;  ? .. 0.09773066926 PowerPlay
//
//
//    /*
//    This assumes your right and left encoders are set up so that driving forward gives you positive ticks on both.
//
//    Start by making sure your inches_Per_Tick is correct. Drive, or even just push, the robot straight forward some known distance (e.g., 120 inches); record how many ticks occurred on the right and left encoders and average them. Then compute inchesPerTick = inches / averageOfRightAndLeftTicks.
//
//    Now (after resetting all encoders) turn the robot in place for a large number of 360 degree turnarounds. For example, 10 complete turns = 20*pi radians. Record the ticks on all three encoders. Use floor tape to make sure you end in the same orientation that you started at.
//
//    WheelBase = (rightTicks - leftTicks) * inchesPerTick / radiansTurned
//
//    HorizontalOffset = horizontalTicks * inchesPerTick / radiansTurned
//     */
//    /*
//    Finaly decided make comments on how I figured out the TICKS_PER_DEGRREE (Note this is a 13.5 inch base. 18inch base should be around 401.6311 depending on wheel position.)
//
//    Turn robot 10 times. (the more times you turn it the more accret it will be.
//    Raw Numbers from first test: L:220774 R:-311937 wheelH:-1836.91724137 (should be 360 x 10=3600)
//    Add R and L = Total T532711
//    Divide T by WheelH = 147.975277778 x 2 = 295.950555556 //should have been Wheelh not 3600
//
//    Test
//            Math didnt work time to guess
//    230 273
//    140 672
//    190 329
//    175 355
//    170 370
//    174.5 359
//    174.34 359.5
//    174.22 360.9
//    174.29 360.4
//
//    2nd test new wheel
//    174.33 370
//    157.3 479
//    200 314
//
//     */
//
//
//    boolean init = false;
//
//    boolean useImu = false;
//
//    double wheelHead;
//
//    double lastImu;
//
//    MovingStatistics speedometer = new MovingStatistics(50);
//
//    long lastUpdateTime;
//
//    double headingOffsetDegrees;
//
//    public void setFirstTrackingVal(double x, double y) {
//        trackingVector.setCartesian(x,y);
//            }
//    public void setHeading(double newHeading) {
//           heading = newHeading;
//
//
//    }
//    public void SetX(double NewX) {
//        x = NewX;
//        trackingVector.setCartesian(x,y);
//    }
//    public void SetY(double NewY) {
//        y = NewY;
//        trackingVector.setCartesian(x,y);
//    }
//    public void ZeroHeading() {
//        lastPosLeft = cPosLeft;
//        lastPosRight = cPosRight;
//    }
//
//
//    public void setHeadingOffsetDegrees(double imuHeading)
//    {
//        this.headingOffsetDegrees = heading - imuHeading ;
//
//        System.out.println(String.format("Sync heading w/ IMU... IMU says %f wheels say %f offset is %f", imuHeading, heading, headingOffsetDegrees));
//    }
//
//    public void update(int cPosLeft, int cPosRight, int cPosAux)
//    {
//        if(!init)
//        {
//            init = true;
//            lastPosLeft = cPosLeft;
//            lastPosRight = cPosRight;
//            lastPosAux = cPosAux;
//            return;
//        }
//
//        dPosLeft  = cPosLeft  - lastPosLeft;
//        dPosRight = cPosRight - lastPosRight;
//        dPosAux   = cPosAux  - lastPosAux;
//
//        heading = (cPosRight-cPosLeft)/TICKS_PER_DEGREE-headingOffsetDegrees;
//
//        dH = (dPosRight-dPosLeft)/TICKS_PER_DEGREE;
//
//        dY = -((dPosLeft+dPosRight)/2.0);
//        dX = dPosAux;
//
//        dY /= TICKS_PER_INCH_NORMAL;
//        dX /= TICKS_PER_INCH_STRAFE;
//
//        dX -= dH * AUX_SPIN_SLIP;
//
//        instantVector.setCartesian(dX, dY);
//
//        trackingVector.addPolar(instantVector.getMag(), heading+instantVector.getDir());
//
//        lastPosLeft = cPosLeft;
//        lastPosRight = cPosRight;
//        lastPosAux = cPosAux;
//
//        oldX = x;
//        oldY = y;
//
//        x = trackingVector.getX();
//        y = -trackingVector.getY();
//
//        if(lastUpdateTime != 0)
//        {
//            double deltaDisplacement = Math.sqrt(Math.pow(x-oldX, 2) + Math.pow(y-oldY, 2));
//            long deltaTime = System.currentTimeMillis() - lastUpdateTime;
//            double speed = deltaDisplacement / (deltaTime/1000.0);
//
//            speedometer.add(speed);
//        }
//
//        lastUpdateTime = System.currentTimeMillis();
//    }
//
//    public void clear()
//    {
//        trackingVector.clear();
//    }
//
//    public double getX()
//    {
//        return x;
//    }
//
//    public double getY()
//    {
//        return y;
//    }
//
//    public double getHeading()
//    {
//        return heading;
//    }
//
//    public double speed()
//    {
//        return speedometer.getMean();
//    }
//
//    public double getWheelHead()
//    {
//        return wheelHead;
//    }
//}