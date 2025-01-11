package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Target;

@Config
public class Outake implements Component{

    private PIDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0.002, i = 0, d = 0;
    public static double f = 0.09;

    public int target = 0;

    private final Servo claw;
    private final Servo wrist, angleLeft, angleRight, position;
    private final DcMotor OuttakeSlideLeft;
    private final DcMotor OuttakeSlideRight;
    private final DcMotor RightFrontMotor;

    private Status status;

    private ElapsedTime elapsedTime = null;


    Telemetry telemetry;
    Init init;

    enum Status {
        WallToFront1 (400),
        WallToFront2 (600),
        WallToFront3 (200),
        Wall (0),
        Front(0);

        private final long time;

        private Status (long time){
            this.time= time;
        }

        public long getTime() {
            return time;
        }
    }

    public Outake(Init init, Telemetry telemetry){

        this.init=init;
        this.telemetry=telemetry;

        claw= init.getClaw();
        wrist= init.getWrist();


        this.OuttakeSlideLeft=init.getOuttakeSlideLeft();
        this.OuttakeSlideRight=init.getOuttakeSlideRight();
        this.RightFrontMotor=init.getRightFrontMotor();
        angleLeft = init.getAngleLeft();
        angleRight = init.getAngleRight();
        position = init.getPosition();
        initializeHardware();
        status = Status.Front;

    }

    public void initializeHardware() {

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        target = 0;

        claw.setPosition(ITDCons.close);

    }

    public void setPID(double p){
        controller.setP(p);
    }

//    public void slidePower(double power) {
//        extension1.setPower(power);
//        extension2.setPower(power);
//    }


    public void openClaw() {
        claw.setPosition(ITDCons.open);
    }

    public void closeClaw() {
        claw.setPosition(ITDCons.close);
    }


    public void pickUpFromWall(){
        target = ITDCons.wallPickupTarget;
        wrist.setPosition(ITDCons.wristBack);
        angleRight.setPosition(ITDCons.angleBack);
        angleLeft.setPosition(ITDCons.angleBack);
        position.setPosition(ITDCons.positionBack);

    }

    public void pickUpFromTransfer(){
        target = ITDCons.transferPickupTarget;
        wrist.setPosition(ITDCons.wristFront);
        position.setPosition(ITDCons.positionFront);
        angleRight.setPosition(ITDCons.angleFront);
        angleLeft.setPosition(ITDCons.angleFront);
    }

    public void scoreSample(){
        target = ITDCons.BucketTarget;
        wrist.setPosition(ITDCons.wristBack);
        position.setPosition(ITDCons.positionBack);
        angleRight.setPosition(ITDCons.angleBack);
        angleLeft.setPosition(ITDCons.angleBack);
    }

    public void scoreSpecimen(){
        status= Status.WallToFront1;

        target = ITDCons.SpecimenTarget;

        angleRight.setPosition(ITDCons.angleMiddle);
        angleLeft.setPosition(ITDCons.angleMiddle);
        elapsedTime = new ElapsedTime();
    }

    public void releaseSpecimen(){
        target= ITDCons.ReleaseTarget;
    }

    public void moveSlide() {

//frontRight
        int rotatePos = RightFrontMotor.getCurrentPosition();
        double pid = controller.calculate(rotatePos, target);
        double lift = pid + f;

        OuttakeSlideLeft.setPower(lift);
        OuttakeSlideRight.setPower(lift);

    }

    public int getTarget(){
        return target;
    }

    public void setTarget(int target){
        this.target = target;
    }

    public int getExtensionPos(){
        return RightFrontMotor.getCurrentPosition();
    }

    public void init(){
//        elbow1.setPosition(ITDCons.diffyInit);
//        elbow2.setPosition(ITDCons.diffyInit);

    }

    public void updateOuttake(){

        switch (status){
            case WallToFront1:
                if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                    position.setPosition(ITDCons.positionFront);
                    angleRight.setPosition(ITDCons.angleMiddle);
                    angleLeft.setPosition(ITDCons.angleMiddle);
                    elapsedTime = new ElapsedTime();
                    status = Status.WallToFront2;
                }
                break;
            case WallToFront2:
                if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                    wrist.setPosition(ITDCons.wristFront);
                    angleLeft.setPosition(ITDCons.angleFront);
                    angleRight.setPosition(ITDCons.angleFront);
                    elapsedTime = new ElapsedTime();
                    status = Status.WallToFront3;
                }
                break;
            case WallToFront3:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
                    elapsedTime = null;
                    status = Status.Front;
                }
                break;
        }
    }


}
