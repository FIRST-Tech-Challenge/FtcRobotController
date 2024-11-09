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
        odoComp.update();
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
            double TotalXDiff = Finalx - ((Pose2D) startPos.get(id)).getX(DistanceUnit.MM);
            double TotalYDiff = Finaly - ((Pose2D) startPos.get(id)).getY(DistanceUnit.MM);
            double XDiff = Finalx - x;
            double YDiff = Finaly - y;
            double XGreater = Math.max(1.0,Math.abs(XDiff));
            double YGreater = Math.max(1.0, Math.abs(YDiff));
            double newXSpeed = XDiff / XGreater;
            double newYSpeed = YDiff / YGreater;
            if(Math.abs(XDiff) <= 1.0){
                newXSpeed = 0.0;
            }
            if(Math.abs(YDiff) <= 1.0){
                newYSpeed = 0.0;
            }
            double headingSpeed = 0.0;
            Pose2D start = (Pose2D) startPos.get(id);
            double TotalHeadDiff = start.getHeading(AngleUnit.DEGREES) - robotPos.getHeading(AngleUnit.DEGREES);
            if(Math.abs(robotPos.getHeading(AngleUnit.DEGREES)-TotalHeadDiff) >= 5.0){
                headingSpeed = robotPos.getHeading(AngleUnit.DEGREES) <= 0.0 ? -0.2: 0.2;
            }
            double currentDiff = Math.sqrt(Math.pow(XDiff,2) + Math.pow(YDiff,2));
            double speed = 0.5;
            boolean less1000 = false;
            boolean less500 = false;
            boolean less250 = false;
            boolean less100 = false;
            if(currentDiff < 1000.0){
                less1000 = true;
            }
            if(currentDiff < 500.0){
                less500 = true;
            }
            if(currentDiff < 250.0){
                less250 = true;
            }
            if(currentDiff < 100.0){
                less100 = true;
            }
            if(less100){
                newYSpeed /= 4.5;
                newXSpeed /= 4.5;
            }
            if(!less100 && less250){
                if(XDiff > YDiff){
                    newYSpeed /= 4;
                }else if(YDiff > XDiff){
                    newXSpeed /= 4;
                }else{
                    newYSpeed /= 4;
                    newXSpeed /= 4;
                }
            }else if(!less250 && less500){
                if(XDiff > YDiff){
                    newYSpeed /= 3;
                }else if(YDiff > XDiff){
                    newXSpeed /= 3;
                }else{
                    newYSpeed /= 3;
                    newXSpeed /= 3;
                }
            }else if(!less500 && less1000){
                if(XDiff > YDiff){
                    newYSpeed /= 2;
                }else if(YDiff > XDiff){
                    newXSpeed /= 2;
                }else{
                    newYSpeed /= 2;
                    newXSpeed /= 2;
                }
            }

            /*double speed = 0.25;
            if(Math.abs(XDiff) <= 10.0 && Math.abs(YDiff) <= 10.0){
                speed = 0.1;
            }else{
                if(Math.abs(XDiff) <= 10.0){
                    newXSpeed /= 4;
                }
                if(Math.abs(YDiff) <= 10.0){
                    newYSpeed /= 4;
                }
            }*/
            if(newXSpeed == 0.0 && newYSpeed == 0.0){
                completed = true;
            }
            movementControler.move(newXSpeed, newYSpeed, headingSpeed,speed);
        } else if(typeIds.get(id) == Rotate){
            double Finalx = finalPos.getX(DistanceUnit.MM);
            double Finaly = finalPos.getY(DistanceUnit.MM);
            double x = robotPos.getX(DistanceUnit.MM);
            double y = robotPos.getY(DistanceUnit.MM);
            double TotalXDiff = Finalx - ((Pose2D) startPos.get(id)).getX(DistanceUnit.MM);
            double TotalYDiff = Finaly - ((Pose2D) startPos.get(id)).getY(DistanceUnit.MM);
            double XDiff = Finalx - x;
            double YDiff = Finaly - y;
            double XGreater = Math.max(1.0,Math.abs(XDiff));
            double YGreater = Math.max(1.0, Math.abs(YDiff));
            double newXSpeed = XDiff / XGreater;
            double newYSpeed = YDiff / YGreater;
            if(Math.abs(XDiff) <= 1.0){
                newXSpeed = 0.0;
            }
            if(Math.abs(YDiff) <= 1.0){
                newYSpeed = 0.0;
            }
            double headingSpeed = 0.0;
            Pose2D start = (Pose2D) startPos.get(id);
            double TotalHeadDiff = start.getHeading(AngleUnit.DEGREES) - robotPos.getHeading(AngleUnit.DEGREES);
            if(TotalHeadDiff-Math.abs(robotPos.getHeading(AngleUnit.DEGREES)) >= 5.0){
                headingSpeed = robotPos.getHeading(AngleUnit.DEGREES) <= 0.0 ? -0.2: 0.2;
            }
            if(Math.abs(robotPos.getHeading(AngleUnit.DEGREES)-TotalHeadDiff) <= 1.0){
                completed = true;
            }
            movementControler.move(newXSpeed, newYSpeed, headingSpeed);
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

    public GoBildaPinpointDriver getOdo() {
        return odoComp;
    }
}
