package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.Components.Misc.ColorDistanceRevV3;

public class Claw {

    private RFServo claw;

    private ColorDistanceRevV3 coneObserver;

    //States:
    //CLAW_CLOSED
    //CLAW_CLOSING
    //CLAW_OPEN
    //CLAW_OPENING

    public enum ClawStates {
        CLAW_CLOSED(true, "CLAW_CLOSED"),
        CLAW_CLOSING(false, "CLAW_CLOSING"),
        CLAW_OPEN(false, "CLAW_OPEN"),
        CLAW_OPENING(false, "CLAW_OPENING");

        boolean status;
        String name;

        ClawStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    //temporary
    private final double CLAW_CONE_DISTANCE = 1;

    //temporary
    private final double CLAW_SERVO_MAX_TICK = 0;

    //temporary
    private final double CLAW_CLOSED_TICK = 0;

    //temporary
    private final double CLAW_STICK_DISTANCE = 1;

    //constructor
    public Claw() {
        //init RFServo & Distance sensor
        claw = new RFServo("claw", CLAW_SERVO_MAX_TICK);

        coneObserver = new ColorDistanceRevV3();
    }

    //close the claw
    public void closeClaw() {
        //no input


        //the state of claw opened has to be true TODO: Boy better see sumthin with distance
        if (CLAW_OPEN.status && getConeDistance() <= CLAW_CONE_DISTANCE) {

        //set servo position
        claw.setPosition(CLAW_CLOSED_TICK);


        //set state of claw closed to true
        CLAW_CLOSED.setStatus(true);

        //log to general robot log that the claw has been closed through function closeClaw()
        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",closeClaw()"
                + ",Claw Closed", true);

        }
    }

    //open the claw
    public void openClaw() {
        //no input


        //the state of claw closed has to be true TODO: refer to line 16
        if (CLAW_CLOSED.status && getConeDistance() <= CLAW_CONE_DISTANCE) {

            //set servo position
            claw.setPosition(0);
            //TODO: need separate CLAW_OPEN_TICK constant?

            //set state of claw open to true
            CLAW_OPEN.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + ",Claw Opened", true);
        }
    }

    //look at and return distance to the nearest cone
    public double getConeDistance() {
        //no input
        //no state conditions
        //execute algorithm for observing
        //no setting state
        //log to general robot log that the cone has been observed through function closeClaw()
        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",getConeDistance()"
                + ",Cone Observed", true);


        //just placeholder so that there are no errors
        return 2.0;
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
}

