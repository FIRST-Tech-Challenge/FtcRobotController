package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.ColorDistanceRevV3;

import java.util.ArrayList;


public class Aligner{

//    private final RFMotor extensionMotor;

//    private final RFMotor intakeMotor;

    private ColorDistanceRevV3 alignerSensor;

    //temporary
    private final double MAX_EXTENSION_TICKS = 1000;

    //temporary
    private final double MIN_EXTENSION_TICKS = 0;

    //temporary
    private final double PRESET_ALIGNER_INTAKE_SPEED = 0.8;

    //temporary
    private final double PRESET_ALIGNER_REVERSE_INTAKE_SPEED = -0.5;

    private final double TICKS_PER_INCH = 955.0/32.0;

    //temporary
    public double CONE_IN_ALIGNER_DISTANCE = 10;

    //temporary
    public double CONE_OUT_OF_ALIGNER_DISTANCE = 30;

    public double alignerMotorLastStopTime = 0;
    //temporary
    public final double ALIGNER_MOTOR_STOP_TIME = 0.1;

    //temporary
    ArrayList<Double> extensionMotorCoefs = new ArrayList<>();

    //States:
    //ALIGNER_SLIDES_EXTENDING
    //ALIGNER_SLIDES_RETRACTING
    //ALIGNER_SLIDES_STILL
    //ALIGNER_INTAKE_STILL
    //ALIGNER_INTAKE_INTAKING
    //ALIGNER_INTAKE_REVERSING

    public enum AlignerStates {
        ALIGNER_SLIDES_EXTENDING(false, "ALIGNER_SLIDES_EXTENDING"),
        ALIGNER_SLIDES_RETRACTING(false, "ALIGNER_SLIDES_RETRACTING"),
        ALIGNER_SLIDES_EXTENDED(false, "ALIGNER_SLIDES_EXTENDED"),
        ALIGNER_SLIDES_RETRACTED(false, "ALIGNER_SLIDES_RETRACTED"),
        ALIGNER_SLIDES_STILL(true, "ALIGNER_SLIDES_STILL"),
        ALIGNER_INTAKE_STILL(true, "ALIGNER_INTAKE_STILL"),
        ALIGNER_INTAKE_INTAKING(false, "ALIGNER_INTAKE_INTAKING"),
        ALIGNER_INTAKE_REVERSING(false, "ALIGNER_INTAKE_REVERSING");

        boolean status;
        String name;

        AlignerStates(boolean value, String name) {
            this.status = value;
            this.name = name;
        }

        public void setStatus(boolean status) {
            this.status = status;
            if(status) {
                for (int i = 0; i < Aligner.AlignerStates.values().length; i++) {
                    if (!AlignerStates.values()[i].status) {
                        Aligner.AlignerStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }


    public Aligner() {

//        extensionMotor = new RFMotor("extensionMotor", DcMotor.RunMode.RUN_USING_ENCODER, true,
//                extensionMotorCoefs, MAX_EXTENSION_TICKS, MIN_EXTENSION_TICKS);
//
//        intakeMotor = new RFMotor("intakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, true,
//                MAX_EXTENSION_TICKS, MIN_EXTENSION_TICKS);
//
//        alignerSensor = op.hardwareMap.get(ColorDistanceRevV3.class,"alignerSensor");
    }


//    //extend the aligner fully
//    public void extendAligner() {
//        //no input TODO:There should be an input of distance, as well as speed, unless you want seperate function for setPosition
//
//
//        //state has to be not already extending aligner TODO:if claw is closed and not yet raised, no extend, this is an
//        //TODO: asynchronous function, it can extend when extending, just not when extended
//
//        if (!(CLAW_CLOSED.status && LIFT_GROUND_JUNCTION.status) && !ALIGNER_SLIDES_EXTENDED.status) {
//
//            //set state extending aligner
//            //set state extending aligner to true
//
//            //set motor velocity to preset speed TODO: Refer to line 9
//            extensionMotor.setPosition(MAX_EXTENSION_TICKS);
//
//
//          if (!ALIGNER_SLIDES_EXTENDING.status) {
////              log to general robot log that it is now extending the aligner through function extendAligner() TODO: Only if not already extending
//              logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",extendAligner()"
//                     + ",Aligner Extending", true);
//          }
//
//        }
//
//    }
//
//    //extend or retract to certain position
//    public void setPosition(double targetpos, double speed) {
//        //input of target position and speed
//
//
//        //state has to be claw closed, not raised, and not fully extended
//
//        if (!(CLAW_CLOSED.status && LIFT_GROUND_JUNCTION.status) && !ALIGNER_SLIDES_EXTENDED.status) {
//
//            //set rfmotor position
//            extensionMotor.setPosition(targetpos);
//
//            //set state extending or retracting based on relative position
//
//            //log to general robot log that it is now extending/retarcting the aligner through function setPosition()
//
//            if (targetpos - extensionMotor.getCurrentPosition() > 0) {
//                //set state aligner extending to true
//
//              if (!ALIGNER_SLIDES_EXTENDING.status) {
//                  logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",setPosition()"
//                          + ",Aligner Extending", true);
//              }
//            }
//
//            else if (targetpos - extensionMotor.getCurrentPosition() < 0) {
//                //set state aligner extending to true
//
//              if (!ALIGNER_SLIDES_RETRACTING.status) {
//                  logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",setPosition()"
//                          + ",Aligner Retracting", true);
//              }
//
//            }
//
//
//        }
//
//
//    }
//
//    //retract the aligner fully
//    public void retractAligner() {
//        //no input
//
//
//        //state has to be not already retracting aligner TODO: same as previous func, this is an async func
//        if ((CLAW_CLOSED.status && !LIFT_GROUND_JUNCTION.status)|| !ALIGNER_SLIDES_RETRACTED.status) {
//
//            //set state retracting aligner
//            //set state retracting aligner to true
//
//            //set motor velocity to preset speed TODO:this MUST be an optimized setPosition function
//            extensionMotor.setPosition(MIN_EXTENSION_TICKS);
//
//
//          if (!ALIGNER_SLIDES_RETRACTING.status) {
////              log to general robot log that it is now retracting the aligner through function extendAligner() TODO: only log once(ik u alrdy kn)
//              logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",retractAligner()"
//                     + ",Aligner Retracting", true);
//          }
//
//        }
//
//
//    }
//
//    //spin the motor to inputted power
//    public void spinAligner(double velocity) {
//        //input of power
//
//        //has to be not already spinning at that velocity
//        if (intakeMotor.getVelocity() != velocity) {
//
//            //set motor velocity to preset speed
//            intakeMotor.setVelocity(velocity);
//
//            //set state spinning aligner to intake to true
//            if (velocity < 0) {
//                ALIGNER_INTAKE_REVERSING.setStatus(true);
//            }
//            if (velocity > 0) {
//                ALIGNER_INTAKE_INTAKING.setStatus(true);
//            }
//            else {
//                ALIGNER_INTAKE_STILL.setStatus(true);
//            }
//
//            //log to general robot log that it is now spinning aligner to intake through function spinAlignerIntake()
//            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
//                    ",spinAlignerIntake(),Aligner Spinning at Velocity: " + velocity, true);
//        }
//    }
//
//
//    //spin the motor to align a cone
//    public void spinAlignerIntake() {
//        //no input
//
//        //state has to be not already spinning it to intake a cone TODO: this is an async func
//        if (!ALIGNER_INTAKE_INTAKING.status) {
//
//            //set motor velocity to preset speed
//            intakeMotor.setVelocity(PRESET_ALIGNER_INTAKE_SPEED);
//
//            //set state spinning aligner to intake to true
//            ALIGNER_INTAKE_INTAKING.setStatus(true);
//
//            //log to general robot log that it is now spinning aligner to intake through function spinAlignerIntake()
//            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
//                    ",spinAlignerIntake(),Aligner Spinning to Intake", true);
//        }
//    }
//
//    //spin the motor to reverse out a cone
//    public void reverseAlignerIntake() {
//        //no input
//
//
//        //state has to be not already spinning it to intake a cone TODO: this is an async func
//        if (!ALIGNER_INTAKE_REVERSING.status) {
//
//            //set motor velocity to preset speed
//            intakeMotor.setVelocity(PRESET_ALIGNER_REVERSE_INTAKE_SPEED);
//
//            //set state spinning aligner to reverse intake to true
//            ALIGNER_INTAKE_REVERSING.setStatus(true);
//
//            //log to general robot log that it is now spinning aligner to reverse intake through function reverseAlignerIntake()
//            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
//                    ",reverseAlignerIntake(),Aligner Spinning to Reverse Intake", true);
//        }
//    }
//
//    //stop the motor
//    public void stopAlignerIntake() {
//        //state has to be not already stopped TODO: this is an async func
//        if (!ALIGNER_INTAKE_STILL.status) {
//
//            //set motor velocity to preset speed
//            intakeMotor.setVelocity(0);
//
//            //set state aligner still to true
//            ALIGNER_INTAKE_STILL.setStatus(true);
//
//            //log to general robot log that it is now stopped through function stopAligner()
//            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
//                    ",stopAligner(),Aligner Stopped", true);
//        }
//    }
//
//    //see if there is a cone
//    public boolean isConeInAligner() {
//        //no input
//        //no state conditions
//        //no setting state
//
//        //log to general robot log that the cone has been observed through function isConeInAligner()
//        logger.log("/RobotLogs/GeneralRobot", "alignerSensor,isConeInAligner()"
//                + ",Cone in Aligner Observed", true);
//
//        //execute algorithm for observing
//
//        return alignerSensor.getSensorDistance() < CONE_IN_ALIGNER_DISTANCE;
//
//    }


}
