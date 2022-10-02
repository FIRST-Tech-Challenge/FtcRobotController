package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_INTAKE_INTAKING;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_INTAKE_STILL;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_SLIDES_EXTENDING;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Components.Aligner.AlignerStates.ALIGNER_SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.liftConstants.LIFT_GROUND_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

import java.util.ArrayList;


public class Aligner{

    private final RFMotor extensionMotor;

    private final RFMotor intakeMotor;

    //temporary
    private final double MAX_EXTENSION_TICKS = 1000;

    //temporary
    private final double MIN_EXTENSION_TICKS = 0;

    private final double PRESET_ALIGNER_INTAKE_SPEED = 0.8;

    private final double PRESET_ALIGNER_REVERSE_INTAKE_SPEED = -0.5;

    private final double DEG_PER_TICK_MOTOR = 18.0/116.0;

    private final double TICKS_PER_INCH = 955.0/32.0;

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
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public Aligner() {

        extensionMotor = new RFMotor("extensionMotor", DcMotor.RunMode.RUN_USING_ENCODER, true,
                extensionMotorCoefs, MAX_EXTENSION_TICKS, MIN_EXTENSION_TICKS);

        intakeMotor = new RFMotor("intakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, true,
                MAX_EXTENSION_TICKS, MIN_EXTENSION_TICKS);
    }


    //extend the aligner fully
    public void extendAligner() {
        //no input TODO:There should be an input of distance, as well as speed, unless you want seperate function for setPosition


        //state has to be not already extending aligner TODO:if claw is closed and not yet raised, no extend, this is an
        //TODO: asynchronous function, it can extend when extending, just not when extended

        if (CLAW_CLOSED.status && LIFT_GROUND_JUNCTION.value == 0 && !ALIGNER_SLIDES_EXTENDED.status) {

            //set state extending aligner
            //set state extending aligner to true

            //set motor velocity to preset speed TODO: Refer to line 9
            extensionMotor.setPosition(MAX_EXTENSION_TICKS);


          if (!ALIGNER_SLIDES_EXTENDING.status) {
//              log to general robot log that it is now extending the aligner through function extendAligner() TODO: Only if not already extending
              logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",extendAligner()"
                     + ",Aligner Extending", true);
          }

        }

    }

    //extend or retract to certain position
    public void setPosition(double targetpos, double speed) {
        //input of target position and speed


        //state has to be claw closed, not raised, and not fully extended

        if (CLAW_CLOSED.status && LIFT_GROUND_JUNCTION.value == 0 && !ALIGNER_SLIDES_EXTENDED.status) {

            //set rfmotor position
            extensionMotor.setPosition(targetpos);

            //set state extending or retracting based on relative position

            //log to general robot log that it is now extending/retarcting the aligner through function setPosition()

            if (targetpos - extensionMotor.getCurrentPosition() > 0) {
                //set state aligner extending to true

              if (!ALIGNER_SLIDES_EXTENDING.status) {
                  logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",setPosition()"
                          + ",Aligner Extending", true);
              }
            }

            else if (targetpos - extensionMotor.getCurrentPosition() < 0) {
                //set state aligner extending to true

              if (!ALIGNER_SLIDES_RETRACTING.status) {
                  logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",setPosition()"
                          + ",Aligner Retracting", true);
              }

            }


        }


    }

    //retract the aligner fully
    public void retractAligner() {
        //no input


        //state has to be not already retracting aligner TODO: same as previous func, this is an async func
        if (CLAW_CLOSED.status && LIFT_GROUND_JUNCTION.value == 0 && !ALIGNER_SLIDES_RETRACTED.status) {

            //set state retracting aligner
            //set state retracting aligner to true

            //set motor velocity to preset speed TODO:this MUST be an optimized setPosition function
            extensionMotor.setPosition(MIN_EXTENSION_TICKS);


          if (!ALIGNER_SLIDES_RETRACTING.status) {
//              log to general robot log that it is now retracting the aligner through function extendAligner() TODO: only log once(ik u alrdy kn)
              logger.log("/RobotLogs/GeneralRobot", extensionMotor.motor.getDeviceName() + ",retractAligner()"
                     + ",Aligner Retracting", true);
          }

        }


    }


    //spin the motor to align a cone
    public void spinAlignerIntake() {
        //no input


        //state has to be not already spinning it to intake a cone TODO: this is an async func
        if (!ALIGNER_INTAKE_INTAKING.status) {

            //set motor velocity to preset speed
            intakeMotor.setVelocity(PRESET_ALIGNER_INTAKE_SPEED);

            //set state spinning aligner to intake to true
            //set state ALIGNER_INTAKE_INTAKING to true

            //log to general robot log that it is now spinning aligner to intake through function spinAlignerIntake()
            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
                    ",spinAlignerIntake(),Aligner Spinning to Intake", true);
        }
    }

    //spin the motor to reverse out a cone
    public void reverseAlignerIntake() {
        //no input


        //state has to be not already spinning it to intake a cone TODO: this is an async func
        if (!ALIGNER_INTAKE_REVERSING.status) {

            //set motor velocity to preset speed
            intakeMotor.setVelocity(PRESET_ALIGNER_REVERSE_INTAKE_SPEED);

            //set state spinning aligner to reverse intake to true
            //set state ALIGNER_INTAKE_REVERSING to true

            //log to general robot log that it is now spinning aligner to reverse intake through function reverseAlignerIntake()
            logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
                    ",reverseAlignerIntake(),Aligner Spinning to Reverse Intake", true);
        }
    }

    //stop the motor
    public void stopAligner() {
        //state has to be not already stopped TODO: this is an async func
        if (!ALIGNER_INTAKE_STILL.status) {

        //set motor velocity to preset speed
        intakeMotor.setVelocity(0);

        //set state aligner still to true
        //set state ALIGNER_INTAKE_STILL to true

        //log to general robot log that it is now stopped through function stopAligner()
        logger.log("/RobotLogs/GeneralRobot", intakeMotor.motor.getDeviceName() +
                ",stopAligner(),Aligner Stopped", true);
        }
    }
}
