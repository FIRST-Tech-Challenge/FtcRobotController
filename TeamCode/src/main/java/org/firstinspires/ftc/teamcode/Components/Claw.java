package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSING;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPENING;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_WIDE;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_WIDING;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Claw {

    private RFServo claw;

    private Rev2mDistanceSensor leftDist;
    private Rev2mDistanceSensor rightDist;


    //temporary
    private final double CLAW_CONE_DISTANCE = 2;

    //temporary
    private final double CLAW_SERVO_MAX_TICK = 1.0;

    //temporary
    private final double CLAW_CLOSED_POS = 0.57;

    //temporary
    private final double CLAW_OPEN_POS = 0.865;

    private final double CLAW_WIDE_POS = 1.0;

    private final double CLAW_Tele_POS = 1.0;


    //temporary
    private final double CLAW_STICK_DISTANCE = 1;
    private double lastCheckTime = 0;

    public double clawServoLastSwitchTime = 0;
    private double lastOpenTime =-10;
    private double clawPos = 0.45;
    //temporary
    public final double CLAW_SERVO_SWITCH_TIME = 0.35;
    boolean shouldUseClawSensor = true;

    //States:
    //CLAW_CLOSED
    //CLAW_CLOSING
    //CLAW_OPEN
    //CLAW_OPENING

    public enum ClawStates {

        CLAW_CLOSED(false, "CLAW_CLOSED"),
        CLAW_CLOSING(false, "CLAW_CLOSING"),
        CLAW_OPEN(true, "CLAW_OPEN"),
        CLAW_OPENING(false, "CLAW_OPENING"),
        CLAW_WIDE(false, "CLAW_WIDE"),
        CLAW_WIDING(false,"CLAW_WIDING");

        boolean status;
        String name;

        ClawStates(boolean p_status, String p_name) {
            this.status = p_status;
            this.name = p_name;
        }

        public void setStatus(boolean p_status) {
            this.status = p_status;
            if(p_status) {
                for (int i = 0; i < ClawStates.values().length; i++) {
                    if (ClawStates.values()[i] != this) {
                        ClawStates.values()[i].status = false;
                    }
                }
            }
        }

        public boolean getStatus() {
            return this.status;
        }
    }

    //constructor
    public Claw() {
        //init RFServo & Distance sensor
        claw = new RFServo("clawServo", CLAW_SERVO_MAX_TICK);

        leftDist = op.hardwareMap.get(Rev2mDistanceSensor.class, "leftDist");
        rightDist = op.hardwareMap.get(Rev2mDistanceSensor.class, "rightDist");


        if(!isTeleop){
            claw.setPosition(CLAW_CLOSED_POS);
            CLAW_CLOSED.setStatus(true);
            clawServoLastSwitchTime=0;
            claw.setFlipTime(-2);
        }
        else{
            claw.setPosition(CLAW_OPEN_POS);
            CLAW_OPEN.setStatus(true);
            clawServoLastSwitchTime=0;
            claw.setFlipTime(-2);

        }
        shouldUseClawSensor = true;
    }
    public void setClawFlipTime(double flipTime){
        claw.setLastTime(flipTime);
    }

    public void updateClawStates() {
        if (CLAW_CLOSING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_CLOSED.setStatus(true);
        }
        if (CLAW_OPENING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_OPEN.setStatus(true);
        }
        if (CLAW_WIDING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_WIDE.setStatus(true);
        }

//        op.telemetry.addData("coneDist",coneObserver.getDistance(INCH));
    }
    public double getLastTime(){
        return claw.getLastTime();
    }

    public void setLastOpenTime(double lastOpenTime) {
        this.lastOpenTime = lastOpenTime;
    }

    public double getLastOpenTime() {
        return lastOpenTime;
    }

    public void logClawStates() {
        logger.log("/RobotLogs/GeneralRobot", CLAW_CLOSED.status + " " + CLAW_OPEN.status, false,
                false, true);
    }

    public void toggleClawPosition() {
        if (claw.flipServoInterval(CLAW_OPEN_POS, CLAW_CLOSED_POS)) {
            clawServoLastSwitchTime = claw.getLastTime();
        }
        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",toggleClawPosition()"
                + ",Claw Toggled", true);
        logger.log("/RobotLogs/GeneralRobot", clawServoLastSwitchTime + " " +
                CLAW_SERVO_SWITCH_TIME + " " + op.getRuntime());
    }
    public void wideClaw(){
        if((CLAW_OPEN.status|| CLAW_CLOSED.status)){
            claw.setPosition(CLAW_WIDE_POS+0.04);
            CLAW_WIDING.setStatus(true);
        }
    }
    public void teleClaw(){
        if(ARM_INTAKE.getStatus()&&!CLAW_WIDE.status){
            claw.setPosition(CLAW_Tele_POS);
            CLAW_WIDING.setStatus(true);
            clawPos = CLAW_WIDE_POS;
        }
    }
    public boolean isClawWide(){
        if(CLAW_WIDE.getStatus()){
            return true;
        }
        else {
            return false;
        }
    }
    //close the claw
    public void closeClaw(Vector2d velo) {
        //no input
        double velocity = velo.getY()* velo.getY() + velo.getX() * velo.getX();
        velocity = sqrt(velocity);
        //the state of claw opened has to be true
        if (op.getRuntime()-lastCheckTime>0.2) {
            lastCheckTime = op.getRuntime();
            if((CLAW_OPEN.status|| CLAW_WIDE.status) && isConeReady(velocity)&& ARM_INTAKE.getStatus()) {
                //set servo position
                claw.setPosition(CLAW_CLOSED_POS);

                //set state of claw closed to true
                CLAW_CLOSING.setStatus(true);

                //log to general robot log that the claw has been closed through function closeClaw()
                logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",closeClaw()"
                        + ",Claw Closed", true);
            }
        }

    }
    public void closeClawRaw() {
        //no input


        //the state of claw opened has to be true
        if (CLAW_OPEN.status|| CLAW_WIDE.status) {
            //set servo position
            claw.setPosition(CLAW_CLOSED_POS);
            clawPos=CLAW_CLOSED_POS;
            //set state of claw closed to true
            CLAW_CLOSING.setStatus(true);

            //log to general robot log that the claw has been closed through function closeClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",closeClaw()"
                    + ",Claw Closed", true);
        }


    }
    //open the claw
    public void openClaw() {
        //no input


        //the state of claw closed has to be true
        if (CLAW_CLOSED.status|| CLAW_WIDE.status) {
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + CLAW_OPENING.status +"clawClosed" + CLAW_CLOSED.getStatus() + "clawWide" + CLAW_WIDE.getStatus(), true);
            //set servo position
            claw.setPosition(CLAW_OPEN_POS);

            //set state of claw open to true
            CLAW_OPENING.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + CLAW_OPENING.status +"clawClosed" + CLAW_CLOSED.getStatus() + "clawWide" + CLAW_WIDE.getStatus(), true);
        }
    }
    public void openClaw(double delay) {
        //no input


        //the state of claw closed has to be true TODO: refer to line 16
        if (CLAW_CLOSED.status) {
            //set servo position
            claw.setPosition(CLAW_OPEN_POS);
            claw.setFlipTime(op.getRuntime()+delay);
            //TODO: need separate CLAW_OPEN_POS constant?

            //set state of claw open to true
            CLAW_OPENING.setStatus(true);
            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + ",Claw Opened", true);
        }
    }


    //look at and return distance to the nearest cone
    public boolean isConeReady(double velocity) {
        //no input
        //no state conditions
        //execute algorithm for observing
        //no setting state
        //log to general robot log that the cone has been observed through function closeClaw()
//        double distance = coneObserver.getDistance(INCH);
//                logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",getConeDistance(),"
//                + coneObserver.getDistance(INCH) + " inches", true);
//        op.telemetry.addData("coneDist",distance);
//        if(distance>2000){
//            shouldUseClawSensor = false;
//        }
        double dist = coneDistance();
        return dist<5||velocity>10&&dist<12;
    }
    public double coneDistance(){
        return leftDist.getDistance(INCH)+rightDist.getDistance(INCH);
    }

    //look at and return distance to the top of the stick
    public boolean getStickDistance() {
        double distancetostick = 0;
        //no input
        //no state conditions
        //execute algorithm for observing
        //no setting state

        //log to general robot log that the cone has been observed through function closeClaw()

        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",getStickDistance()"
                + ",Stick Observed", true);

        if (distancetostick <= CLAW_STICK_DISTANCE) {
            return true;
        }
        else {
                return false;
        }
    }

    public double getPosition() {
        return claw.getPosition();
    }
}

