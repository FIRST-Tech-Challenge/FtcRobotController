package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSING;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPENING;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Claw {

    private RFServo claw;

    private Rev2mDistanceSensor coneObserver;

    //temporary
    private final double CLAW_CONE_DISTANCE = 2.45;

    //temporary
    private final double CLAW_SERVO_MAX_TICK = 1.0;

    //temporary
    private final double CLAW_CLOSED_POS = 0.45;

    //temporary
    private final double CLAW_OPEN_POS = 0.54;

    private final double CLAW_WIDE_POS = 0.64;

    //temporary
    private final double CLAW_STICK_DISTANCE = 1;
    private double lastCheckTime = 0;

    public double clawServoLastSwitchTime = 0;
    private double lastOpenTime =-10;
    private double clawPos = 0.45;
    //temporary
    public double CLAW_SERVO_SWITCH_TIME = 0.3;
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
        CLAW_OPENING(false, "CLAW_OPENING");

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

        coneObserver = op.hardwareMap.get(Rev2mDistanceSensor.class, "coneObserver");

        if(!isTeleop){
            claw.setPosition(CLAW_CLOSED_POS);
            CLAW_CLOSED.setStatus(true);
            clawServoLastSwitchTime=0;
            claw.setFlipTime(-2);
        }
        else{
            claw.setPosition(CLAW_OPEN_POS);
            CLAW_OPEN.setStatus(true);
            CLAW_SERVO_SWITCH_TIME = 0.3;
        }
        shouldUseClawSensor = true;
    }
    public void setClawFlipTime(double flipTime){
        CLAW_SERVO_SWITCH_TIME = flipTime;
    }

    public void updateClawStates() {
        if (CLAW_CLOSING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_CLOSED.setStatus(true);
        }
        if (CLAW_OPENING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_OPEN.setStatus(true);
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
        if(!CLAW_OPENING.getStatus()&&clawPos!=CLAW_WIDE_POS){
            claw.setPosition(CLAW_WIDE_POS);
            CLAW_OPENING.setStatus(true);
            clawPos = CLAW_WIDE_POS;
        }
    }
    public boolean isClawWide(){
        if(CLAW_OPEN.getStatus()&&clawPos==CLAW_WIDE_POS){
            return true;
        }
        else {
            return false;
        }
    }
    //close the claw
    public void closeClaw() {
        //no input


        //the state of claw opened has to be true
        if (op.getRuntime()-lastCheckTime>0.2 && op.getRuntime()-lastOpenTime>3) {
            lastCheckTime = op.getRuntime();
            if(CLAW_OPEN.status && isConeReady()&& LiftArm.liftArmStates.ARM_INTAKE.getStatus()) {
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
        if (CLAW_OPEN.status) {
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


        //the state of claw closed has to be true TODO: refer to line 16
        if (CLAW_CLOSED.status) {
            //set servo position
            claw.setPosition(CLAW_OPEN_POS);
            //TODO: need separate CLAW_OPEN_POS constant?

            //set state of claw open to true
            CLAW_OPENING.setStatus(true);
            clawPos= CLAW_OPEN_POS;
            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + ",Claw Opened", true);
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
            clawPos= CLAW_OPEN_POS;
            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + ",Claw Opened", true);
        }
    }


    //look at and return distance to the nearest cone
    public boolean isConeReady() {
        //no input
        //no state conditions
        //execute algorithm for observing
        //no setting state
        //log to general robot log that the cone has been observed through function closeClaw()
        double distance = coneObserver.getDistance(INCH);
                logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",getConeDistance(),"
                + coneObserver.getDistance(INCH) + " inches", true);
        op.telemetry.addData("coneDist",distance);
        if(distance>2000){
            shouldUseClawSensor = false;
        }
        return coneObserver.getDistance(INCH) < CLAW_CONE_DISTANCE && shouldUseClawSensor;
    }
    public double coneDistance(){
        return coneObserver.getDistance(INCH);
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

