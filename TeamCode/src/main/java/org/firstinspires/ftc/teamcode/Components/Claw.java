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
    private final double CLAW_CONE_DISTANCE = 2.6;

    //temporary
    private final double CLAW_SERVO_MAX_TICK = 1.0;

    //temporary
    private final double CLAW_CLOSED_POS = 0.31;

    //temporary
    private final double CLAW_OPEN_POS = 0.48;

    //temporary
    private final double CLAW_STICK_DISTANCE = 1;
    private double lastCheckTime = 0;

    public double clawServoLastSwitchTime = 0;
    //temporary
    public final double CLAW_SERVO_SWITCH_TIME = 0.2;

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
            CLAW_OPEN.setStatus(true);
            closeClaw();
        }
        else{
            claw.setPosition(CLAW_OPEN_POS);
            CLAW_OPEN.setStatus(true);
        }
    }

    public void updateClawStates() {
        if (CLAW_CLOSING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_CLOSED.setStatus(true);
        }
        if (CLAW_OPENING.status && op.getRuntime() - claw.getLastTime() > CLAW_SERVO_SWITCH_TIME) {
            CLAW_OPEN.setStatus(true);
        }
        op.telemetry.addData("coneDist",coneObserver.getDistance(INCH));
    }
    public double getLastTime(){
        return claw.getLastTime();
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

    //close the claw
    public void closeClaw() {
        //no input


        //the state of claw opened has to be true
        if (op.getRuntime()-lastCheckTime>0.2) {
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
        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",getConeDistance()"
                + coneObserver.getDistance(INCH), true);
        op.telemetry.addData("coneDist",coneObserver.getDistance(INCH));

        return coneObserver.getDistance(INCH) < CLAW_CONE_DISTANCE;
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

