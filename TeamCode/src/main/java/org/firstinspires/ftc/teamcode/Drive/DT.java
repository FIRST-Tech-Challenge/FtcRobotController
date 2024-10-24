package org.firstinspires.ftc.teamcode.Drive;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Drive.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Math.T;

import java.util.ArrayList;

@Config
public class NewDT{

    private boolean forceStop = false;
    private SampleMecanumDrive drive;

    private DcMotorEx leftFront, rightRear, rightFront, leftRear;

    private double xTarget, yTarget, rTarget;
    private double xRn, yRn, rRn;
    private double deltaX, deltaY, deltaR;
    private boolean isAtTarget =false;



    public double turnVelocity;

    private boolean purePersuiting = false;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    private boolean ending = false;

    private double maxPower = 1;
    private double flPower, frPower, blPower, brPower;

    private double normalize;
    private boolean on = true;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    double lastTime = 0;

    double xVelocity=0;
    double yVelocity = 0;
    public static double rkp=1.0;
    public static double xykp = 0.075;
    double pathController;
    public static double xykd = 0;
    public static double rkd=0.0008;

    double deltaRRateOfChange = 0;
    double lastDeltaR = 0;

    double deltaXRateOfChange = 0;
    double lastDeltaX = 0;

    double deltaYRateOfChange = 0;
    double lastDeltaY = 0;

    double deltaTime = 0;
    double pathLength=0;
    double lastPathLength=0;
    double deltaPathRateOfChange=0;
    public static double pkp = 0.13;
    public static double pkd =0.0005;
    public static double yMultiplier = 1;
    public static double xMultiplier = 1;
    public static double zeroMoveAngle = 45;
    //pure pursuit vars start here
    private double moveRadius;
    private double headingRadius;
    private double movePower;
    private  ArrayList<Pose2d> Waypoints;
    private Pose2d lastDrivePoint;
    private Pose2d lastHeadingPoint;
    private boolean velExtr;
    private Pose2d followDrive, followHeading;
    private double headingOffset;

    public NewDT(HardwareMap hardwareMap, Pose2d startPose, ElapsedTime timer){
        this.timer = timer;
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(startPose);

        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();
    }

    public void setPowers(double y, double x, double r){
        if(!forceStop){
            normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            flPower = (y+x-r);
            blPower = (y-x+r);
            brPower = (y+x+r);
            frPower = (y-x-r);
            leftFront.setPower((flPower/normalize));
            leftRear.setPower((blPower/normalize));
            rightRear.setPower((brPower/normalize));
            rightFront.setPower((frPower/normalize));
        }else{
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
    public void update(){
        deltaTime = timeWrapper(timer.nanoseconds()) - lastTime;
        //change in time

        drive.updatePoseEstimate();
        xRn = drive.getPoseEstimate().getX();
        yRn = drive.getPoseEstimate().getY();
        rRn = drive.getPoseEstimate().getHeading();
        //position updates

        if (Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
        lastAngle = rRn;
        twistedR = count * (2 * M.PI) + rRn;
        //heading wrapper

        deltaR = -1 * (rTarget - twistedR);
        deltaX = xTarget - xRn;
        deltaY = -1 * (yTarget - yRn);

        //deltas


        deltaRRateOfChange = -1 * (deltaR - lastDeltaR) / (deltaTime);
        deltaYRateOfChange = -1 * (deltaY - lastDeltaY) / (deltaTime);
        deltaXRateOfChange = (deltaX - lastDeltaX) / (deltaTime);
        deltaPathRateOfChange = (pathLength - lastPathLength)/ (deltaTime);
        //rate of change of deltas (used for derivative in PID)

        xVelocity = deltaX / deltaTime;
        yVelocity = deltaY / deltaTime;
        turnVelocity = deltaR / deltaTime;
        //for velocity extrapolation, make sure they are in the right direction and right units



        //if statements start

        //pure persuit
        xOut = deltaX;
        yOut = deltaY;
        //literally gets deltas

        xOut/=moveRadius;
        yOut/=moveRadius;
        pathController = pathLength *pkp + deltaPathRateOfChange*pkd;
        if(maxPower<pathController){
            pathController = maxPower;

        }



        rOut = (deltaR * rkp - deltaRRateOfChange * rkd);
        if (Math.abs(rOut) > DTConstants.maxAngularPower)
            rOut = DTConstants.maxAngularPower * Math.signum(rOut);
        //heading power

        // rotation matrix
        xPower = (xOut * T.cos(rRn) - yOut * T.sin(rRn));
        yPower = (xOut * T.sin(rRn) + yOut * T.cos(rRn));





        double errorScale = 1 - (Math.abs(deltaR) / Math.toRadians(zeroMoveAngle));
        if (errorScale < 0) {
            errorScale = 0;
        }
        xPower *= errorScale;
        yPower *= errorScale;
        //if heading error is big, then drive slows down - make sure deltaR is not reversed cuz i changed sum stuff up


        // if powers are over 1, then we get the percentage of xPower and yPower and scale translational movements down by that percentage and weigh heading more
        if(Math.abs(xPower) + Math.abs(yPower) + Math.abs(rOut) > 1.0) {

            double translationPower = 1.0 - Math.abs(rOut);
            double xPercent = Math.abs(xPower) / (Math.abs(xPower) + Math.abs(yPower));

            xPower = Math.signum(xPower) * translationPower * xPercent;
            yPower = Math.signum(yPower) * translationPower * (1 - xPercent);

        }



        setPowers(yPower*pathController*yMultiplier, xPower*pathController*xMultiplier, -rOut);
        //actually sets power to the motor




        lastTime = timeWrapper(timer.nanoseconds());
        lastDeltaR = deltaR;
        lastDeltaX = deltaX;
        lastDeltaY = deltaY;
        lastPathLength = pathLength;
        //updating the last values

    }
    public double getPathController(){
        return pathController;
    }
    public void setPurePersuiting(boolean isPurePersuiting){
        purePersuiting = isPurePersuiting;
    }
    public double timeWrapper(double nano){
        return nano/1000000000;
    }
    public Pose2d getLocation(){
        return new Pose2d(xRn, yRn, rRn);
    }
    public void setXTarget(double x){
        this.xTarget = x;
    }
    public double getCurrentTime(){
        return currentTime;
    }
    public double getTimer(){
        return timer.nanoseconds();
    }
    public void setYTarget(double y){
        this.yTarget = y;
    }
    public void setRTarget(double r){
        this.rTarget = r;
    }
    public double getPowerX(){
        return xPower;
    }
    public double getPowerY(){
        return yPower;
    }
    public double getTurnVeloctiy(){return turnVelocity;}
    public double getPowerR(){
        return rOut;
    }
    public double getRawY(){
        return yOut;
    }
    public double getRawX(){
        return xOut;
    }
    public double getX(){
        return xRn;
    }
    public double getY(){
        return yRn;
    }
    public double getR(){
        return twistedR;
    }
    public double getYTarget(){
        return yTarget;
    }
    public double getXTarget(){
        return xTarget;
    }
    public double getRTarget(){
        return rTarget;
    }
    public double getUnTwistedR(){
        return rRn;
    }
    public double getDeltaX(){
        return deltaX;
    }
    public double getDeltaY(){
        return deltaY;
    }
    public double getDeltaR(){
        return deltaR;
    }
    //i think velocity is in ms, not 100% sure
    public Pose2d getFuturePos(int milliseconds){
        double r1 = xVelocity/turnVelocity;
        double r2 = yVelocity/turnVelocity;

        double relDeltaX = Math.sin(deltaR) * r1 - (1.0 - Math.cos(deltaR)) * r2;
        double relDeltaY = (1.0 - Math.cos(deltaR)) * r1 + Math.sin(deltaR) * r2;

        return new Pose2d(xRn+relDeltaX, yRn+relDeltaY);
    }
    public boolean isAtTarget(){
        return isAtTarget;
    }
    public boolean isAtTargetX(){
        return Math.abs(xRn - xTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetY(){
        return Math.abs(yRn - yTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetR(){
        return Math.abs(twistedR - rTarget) < DTConstants.allowedAngularError;
    }
    public double getTwistedR(){
        return 0;
    } //??
    public void setPoseEstimate(Pose2d pose){
        this.drive.setPoseEstimate(pose);
    }
    public void lineTo(double x, double y, double r){
        setXTarget(x);
        setYTarget(y);
        setRTarget(r);
    }
    public void lineToCHeading(double x, double y){
        setXTarget(x);
        setYTarget(y);
    }
    public void lineToChangeHeadingUnderCondition(double x, double y, double r, boolean condition){
        setXTarget(x);
        setYTarget(y);
        if(condition){
            setRTarget(r);
        }
    }
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }
    public void setOn(boolean a){
        this.on = a;
    }

    public void finishPurePersuiting(Pose2d endPoint){
        purePersuiting = false;
        setXTarget(endPoint.getX());
        setYTarget(endPoint.getY());
        setRTarget(endPoint.getHeading());
    }

    //Takes in current posx, posy, posr, targetx, and targety
    //calculates the angle to follow that requires the least rotation
    public double toPoint(double x, double y, double r, double x2, double y2){
        double x3 = x2 - x;
        double y3 = y2 - y;

        double x4 = Math.cos(r);
        double y4 = Math.sin(r);

        double dotProduct = x3 * x4 + y3 * y4;

        double magnitude = Math.sqrt(x3 * x3 + y3 * y3);
        double cosine = Math.acos(dotProduct / magnitude);
        double angle = Math.atan2(y3, x3);
        double finalAngle=0;

        if(nearlyEqual(Math.cos(angle), Math.cos(r+cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r+cosine))){
            finalAngle = r+cosine;
        }
        if(nearlyEqual(Math.cos(angle), Math.cos(r-cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r-cosine))){
            finalAngle = r-cosine;
        }


        return finalAngle;
    }
    public boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }

    public void setForceStop(boolean b){
        forceStop =b;
    }
    public boolean isForceStopped(){
        return forceStop;
    }

    public void setPathEndHold(boolean a){
        this.ending = a;
    }


    public boolean withinPoint(Pose2d endpoint, double range){
        double value = Math.hypot(getX()-endpoint.getX(), getY()-endpoint.getY());
        return value < range;
    }
    //setter method for pure pursuit constants
    public void setPPConstants(double mr, double hr, double mp){
        moveRadius = mr;
        headingRadius = hr;
        movePower = mp;
        on = true;
    }
    //every segment of nonstop movement(aka accounts for break for carryinh out other actions), assuming new path
    public void newPath(ArrayList<Pose2d> wayPoints, double headingOffset, boolean velExtr){
        Waypoints = wayPoints;
        this.headingOffset = headingOffset;
        PurePursuitUtil.updateMoveSegment(1);
        PurePursuitUtil.updateHeadingSegment(1);
        lastDrivePoint = wayPoints.get(0);
        lastHeadingPoint = wayPoints.get(0);
        this.velExtr = velExtr;
    }
    //actullay pp execution
    public void followDrive(){
        if(velExtr){
            followDrive = PurePursuitUtil.followMe(new ArrayList<>(Waypoints), getFuturePos(500), moveRadius, lastDrivePoint, false);
            lastDrivePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(new ArrayList<>(Waypoints), getFuturePos(500), headingRadius, lastHeadingPoint, true);
            lastHeadingPoint = followHeading;
        }
        else{
            followDrive = PurePursuitUtil.followMe(new ArrayList<>(Waypoints), getLocation(), moveRadius, lastDrivePoint, false);
            lastDrivePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(new ArrayList<>(Waypoints), getLocation(), headingRadius, lastHeadingPoint, true);
            lastHeadingPoint = followHeading;
            lineTo(followDrive.getX(), followDrive.getY(),toPoint(getX(), getY(), getR(), followHeading.getX(), followHeading.getY())+headingOffset);
            pathLength = Math.hypot((Waypoints.get(PurePursuitUtil.getMoveSegment()).getX()-getX()), (Waypoints.get(PurePursuitUtil.getMoveSegment()).getY()-getY()));
            for(int i=PurePursuitUtil.getMoveSegment()+1;i<Waypoints.size();i++){
                pathLength += Math.hypot((Waypoints.get(i-1).getX()-Waypoints.get(i).getX()), (Waypoints.get(i-1).getY()-Waypoints.get(i).getY()));
            }
            this.pathLength = pathLength;
            maxPower = movePower;
            update();
        }
    }
}
