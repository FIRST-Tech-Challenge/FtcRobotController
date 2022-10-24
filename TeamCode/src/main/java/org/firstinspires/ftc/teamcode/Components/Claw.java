package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.Components.Misc.ColorDistanceRevV3;

public class Claw {

    private RFServo claw;

    private RevColorSensorV3 coneObserver;

    //temporary
    private final double CLAW_CONE_DISTANCE = 0.75;

    //temporary
    private final double CLAW_SERVO_MAX_TICK = 1.0;

    //temporary
    private final double CLAW_CLOSED_POS = 0.3;

    //temporary
    private final double CLAW_OPEN_POS = 0.4;

    //temporary
    private final double CLAW_STICK_DISTANCE = 1;

    public double clawServoLastSwitchTime = 0;
    //temporary
    public final double CLAW_SERVO_SWITCH_TIME = 1;

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
    }

    //constructor
    public Claw() {
        //init RFServo & Distance sensor
        claw = new RFServo("clawServo", CLAW_SERVO_MAX_TICK);

        coneObserver = op.hardwareMap.get(RevColorSensorV3.class, "coneObserver");
    }

    public void logClawStates() {
        logger.log("/RobotLogs/GeneralRobot", CLAW_CLOSED.status + " " + CLAW_OPEN.status, false,
                false, true);
    }

    public void toggleClawPosition() {
        if (claw.flipServoInterval(CLAW_OPEN_POS, CLAW_CLOSED_POS)) {
            clawServoLastSwitchTime = claw.getlasttime();
        }
        logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",toggleClawPosition()"
                + ",Claw Toggled", true);
        logger.log("/RobotLogs/GeneralRobot", clawServoLastSwitchTime + " " +
                CLAW_SERVO_SWITCH_TIME + " " + op.getRuntime());
    }

    //close the claw
    public void closeClaw() {
        //no input


        //the state of claw opened has to be true TODO: Boy better see sumthin with distance
        if (CLAW_OPEN.status && isConeReady()) {
            //set servo position
            claw.setPosition(CLAW_CLOSED_POS);

            //set state of claw closed to true
            CLAW_CLOSED.setStatus(true);

            //log to general robot log that the claw has been closed through function closeClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",closeClaw()"
                    + ",Claw Closed", true, true);
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
            CLAW_OPEN.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", claw.getDeviceName() + ",openClaw()"
                    + ",Claw Opened", true, true);
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
                + ",Cone in Claw Observed", true);

        return coneObserver.getDistance(INCH) < CLAW_CONE_DISTANCE;
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

