package com.parshwa.drive.auto;

import static com.parshwa.drive.auto.Types.*;

import com.parshwa.drive.tele.Drive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class AutoDriverBetaV1 implements AutoDriverInterface {
    private GoBildaPinpointDriver odoComp;
    private Drive movementControler;
    private ArrayList movementIds = new ArrayList();
    private ArrayList typeIds = new ArrayList();
    private ArrayList startPos = new ArrayList();
    private boolean stay = false;
    @Override
    public void init(HardwareMap hwMP, Drive movementController) {
        this.odoComp = hwMP.get(GoBildaPinpointDriver.class,"odo");
        odoComp.setOffsets(-48.0, 168.0);
        odoComp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoComp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odoComp.resetPosAndIMU();
        this.movementControler = movementController;
    }

    @Override
    public Pose2D getPosition() {
        Pose2D pos = odoComp.getPosition();
        return new Pose2D(DistanceUnit.MM,pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public double getVelocity(Directions direction) {
        Pose2D vel = odoComp.getVelocity();
        switch (direction){
            case XDirection:
                return vel.getX(DistanceUnit.MM);
            case YDirection:
                return vel.getY(DistanceUnit.MM);
            case HeadingDirection:
                return vel.getHeading(AngleUnit.DEGREES);
            default:
                return 0.0;
        }
    }

    @Override
    public int lineTo(double XEnd, double YEnd, double MaxVelocity) {
        Pose2D robotPos = getPosition();
        Pose2D finalPos = new Pose2D(DistanceUnit.MM, XEnd, YEnd, AngleUnit.DEGREES, robotPos.getHeading(AngleUnit.DEGREES));
        int id = getId();
        movementIds.add(id,finalPos);
        typeIds.add(id, LineTo);
        startPos.add(id, robotPos);
        return id;
    }
    private int getId(){
        int id = movementIds.size();
        return id;
    }
    public boolean move(int id){
        odoComp.update();
        boolean completed = false;
        Pose2D finalPos = (Pose2D) movementIds.get(id);
        Pose2D robotPos = odoComp.getPosition();
        if(typeIds.get(id) == LineTo){
            double Finalx = finalPos.getX(DistanceUnit.MM);
            double Finaly = finalPos.getY(DistanceUnit.MM);
            double x = robotPos.getX(DistanceUnit.MM);
            double y = robotPos.getY(DistanceUnit.MM);
            double XDiff = Finalx - x;
            double YDiff = Finaly - y;
            double maximum = Math.max(1.0,Math.abs(XDiff));
            maximum = Math.max(maximum, Math.abs(YDiff));
            double newXSpeed = XDiff / maximum;
            double newYSpeed = YDiff / maximum;
            if(Math.abs(XDiff) <= 7.5){
                newXSpeed = 0.0;
            }
            if(Math.abs(YDiff) <= 7.5){
                newYSpeed = 0.0;
            }
            double headingSpeed = 0.0;
            Pose2D start = (Pose2D) startPos.get(id);
            double TotalHeadDiff = start.getHeading(AngleUnit.DEGREES) - robotPos.getHeading(AngleUnit.DEGREES);
            if(Math.abs(robotPos.getHeading(AngleUnit.DEGREES)-TotalHeadDiff) >= 5.0){
                headingSpeed = robotPos.getHeading(AngleUnit.DEGREES) <= 0.0 ? -0.001: 0.0001;
            }
            double speed = 0.75;
            double currentDiff = Math.sqrt(Math.pow(XDiff,2) + Math.pow(YDiff,2));
            if(currentDiff <= 1000.0){
                speed = 0.65;
            }
            if(currentDiff <= 500.0){
                speed = 0.45;
            }
            if(currentDiff <= 100.0){
                speed = 0.25;
            }
            if(currentDiff <= 50.0){
                speed = 0.15;
            }
            speed = 0.3333333333;
            if(newXSpeed == 0.0 && newYSpeed == 0.0){
                completed = true;
            }
            movementControler.move(newXSpeed, newYSpeed, headingSpeed,speed);
        } else if(typeIds.get(id) == Rotate){
            double currentHed = robotPos.getHeading(AngleUnit.DEGREES);
            double targetHed = finalPos.getHeading(AngleUnit.DEGREES);
            currentHed = AngleUnit.normalizeDegrees(currentHed);
            targetHed = AngleUnit.normalizeDegrees(targetHed);
            double headingDiff = targetHed - currentHed;
            double headingSpeed = Math.abs(headingDiff) >= 1.0 ? headingDiff / Math.abs(headingDiff) : headingDiff;
            if(Math.abs(headingDiff) >= 180.0 || stay){
                headingSpeed = -headingSpeed;
                stay = true;
            }
            double speed = 0.1;
            if(targetHed <= currentHed + 10.0 && targetHed >= currentHed - 10.0){
                completed = true;
                stay = false;
                movementControler.move(0.0,0.0,0.0,0.5);
            }else{
                movementControler.move(0.0,0.0,headingSpeed,speed);
            }
        } else if(typeIds.get(id) == CurveTo){
            completed = true;
        } else {
            completed = true;
        }
        return completed;
    }

    @Override
    public List<Integer> curveTo(double CircleCenterX, double CircleCenterY, double TotalAngle, double MaxAngleVelocity, Directions curveDirection) {
        List<Integer> ids = new ArrayList<>();
        return ids;
    }
    @Override
    public int rotateRobot(double degrees, Directions rotationDirection) {
        Pose2D robotPos = getPosition();
        Pose2D finalPos = new Pose2D(DistanceUnit.MM, robotPos.getX(DistanceUnit.MM), robotPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, degrees);
        int id = getId();
        movementIds.add(id,finalPos);
        typeIds.add(id, Rotate);
        startPos.add(id, robotPos);
        return id;
    }
}
