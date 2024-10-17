package com.parshwa.drive.auto;

import static com.parshwa.drive.auto.Directions.HeadingDirection;
import static com.parshwa.drive.auto.Directions.XDirection;
import static com.parshwa.drive.auto.Directions.YDirection;
import static com.parshwa.drive.auto.Types.*;

import android.text.style.TtsSpan;

import com.parshwa.drive.tele.Drive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.sql.Array;
import java.util.ArrayList;
import java.util.function.Function;

public class AutoDriverBetaV1 implements AutoDriverInterface {
    private GoBildaPinpointDriver odoComp;
    private Drive movementControler;
    private ArrayList movementIds = new ArrayList();
    private ArrayList typeIds = new ArrayList();
    @Override
    public void init(HardwareMap hwMP, Drive movementController) {
        this.odoComp = hwMP.get(GoBildaPinpointDriver.class,"computer");
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
                break;
            case YDirection:
                return vel.getY(DistanceUnit.MM);
                break;
            case HeadingDirection:
                return vel.getHeading(AngleUnit.DEGREES);
                break;
        }
    }

    @Override
    public int lineTo(double XEnd, double YEnd, double MaxVelocity) {
        Pose2D robotPos = getPosition();
        Pose2D finalPos = new Pose2D(DistanceUnit.MM, XEnd, YEnd, AngleUnit.DEGREES, robotPos.getHeading(AngleUnit.DEGREES));
        int id = getId();
        movementIds.add(id,finalPos);
        typeIds.add(id, LineTo);
        return id;
    }
    private int getId(){
        int id = movementIds.size();
        return id;
    }
    public boolean move(int id){
        boolean completed = false;
        Pose2D finalPos = (Pose2D) movementIds.get(id);
        if(typeIds.get(id) == LineTo){
            Pose2D robotPos = odoComp.getPosition();
            double Finalx = finalPos.getX(DistanceUnit.MM);
            double Finaly = finalPos.getY(DistanceUnit.MM);
            double x = robotPos.getX(DistanceUnit.MM);
            double y = robotPos.getY(DistanceUnit.MM);
            double XDiff = Finalx - x;
            double YDiff = Finaly - y;
            double maximum = Math.max(1,Math.abs(XDiff));
            maximum = Math.max(maximum, Math.abs(YDiff));
            double newXSpeed = XDiff / maximum;
            double newYSpeed = YDiff / maximum;
            movementControler.move();
        } else if(typeIds.get(id) == Rotate){

        } else if(typeIds.get(id) == CurveTo){
            completed = true;
        } else {
            completed = true;
        }
        return completed;
    }

    @Override
    public int curveTo(double CircleCenterX, double CircleCenterY, double TotalAngle, double MaxAngleVelocity, Directions curveDirection) {
        return 0;
    }
    @Override
    public int rotateRobot(double degrees, Directions rotationDirection) {
        Pose2D robotPos = getPosition();
        Pose2D finalPos;
        if(Directions.LeftRotateDirection == rotationDirection){
            finalPos = new Pose2D(DistanceUnit.MM, robotPos.getX(DistanceUnit.MM), robotPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, robotPos.getHeading(AngleUnit.DEGREES)+degrees);
        } else if(Directions.RightRotateDirection == rotationDirection){
            finalPos = new Pose2D(DistanceUnit.MM, robotPos.getX(DistanceUnit.MM), robotPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, robotPos.getHeading(AngleUnit.DEGREES)-degrees);
        } else {
            finalPos = new Pose2D(DistanceUnit.MM, robotPos.getX(DistanceUnit.MM), robotPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, robotPos.getHeading(AngleUnit.DEGREES));
        }
        int id = getId();
        movementIds.add(id,finalPos);
        typeIds.add(id, Rotate);
        return id;
    }
}
