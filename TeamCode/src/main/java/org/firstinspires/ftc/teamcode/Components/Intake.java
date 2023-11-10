package org.firstinspires.ftc.teamcode.Components;


import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.openftc.easyopencv.LIFO_OpModeCallbackDelegate;

/**
 * Warren
 * Class to contain flipping intake and associated functions
 */
@Config
public class Intake extends RFMotor {
    RFBreakBeam breakBeam;
    RFLimitSwitch limitSwitch;
    Motor motor;

    private final double INTAKE_POWER = 0.6;
    private final double REVERSE_POWER=-0.3;

    private boolean full = false;
    private double pixelCount =0;
    public static double HALF_TICKS_PER_REV = 383.6/2;

    /**
     * initializes all the hardware, logs that hardware has been initialized
     */
    public Intake(){
        super("intakeMotor",true);
        super.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        super.setDirection(DcMotorSimple.Direction.REVERSE);
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("Initializing Intake Motor and intake sensors!");
//        breakBeam = new RFBreakBeam();
//        limitSwitch = new RFLimitSwitch("intakeSwitch");
    }

    /**
     * possible states of intake
     */
    public enum IntakeStates{
        STOPPED(true),
        INTAKING(false),
        REVERSING(false);


        boolean state;
        IntakeStates(boolean p_state){
            state = p_state;
        }

        /**
         * sets current state to true, logs that this state is true in general and intake surface
         */
        public void setStateTrue() {
            if(!this.state) {
                for (int i = 0; i < IntakeStates.values().length; i++) {
                    IntakeStates.values()[i].state = false;
                }
                LOGGER.log("Intake state changed to: " + this.name());
                this.state = true;
            }
        }
        public boolean getState(){
            return this.state;
        }
    }

    /**
     * Sets intake power to INTAKE_POWER, logs that the robot is intaking to general and intake surface level
     */
    public void intake(){
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("starting intake, power : "+INTAKE_POWER);
        setPower(INTAKE_POWER);
        IntakeStates.INTAKING.setStateTrue();
    }
    /**
     * Sets intake power to REVERSE_POWER, logs that the robot is reversing to general and intake surface level
     */
    public void reverseIntake(){
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("reversing intake, power : "+REVERSE_POWER);
        setPower(REVERSE_POWER);
        IntakeStates.REVERSING.setStateTrue();

    }
    /**
     * Sets intake power 0, logs that intake is stopped to general and intake surface level
     */
    public void stopIntake(){
        double vel = super.getVelocity();
//        if(abs(vel)>10) {
            double pos = super.getCurrentPosition()-20;
            double res = (pos) % HALF_TICKS_PER_REV;
            if (res > HALF_TICKS_PER_REV / 2.0) {
                res -= HALF_TICKS_PER_REV;
            }
            if (abs(res) < 20) {
                LOGGER.log(RFLogger.Severity.FINE, "stopping intake, power : " + 0 +", "+ res);
                setPower(0);
            } else {
            }
//        }
//        LOGGER.log("position" + pos);
        IntakeStates.STOPPED.setStateTrue();
    }

    /**
     * count number of pixels using sensor, log to intake and general surface level when change
     * @return number of pixels in intake
     */
    public int countPixels(){
        int count=0;
//        if(limitSwitch.isSwitched()){
//            count++;
////            break beam here
//            if(true){
//                count++;
//            }
//        }
//        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
//        LOGGER.log("Intake.countPixels() : count : "+pixelCount + " --> " + count);
//        if(count!=pixelCount){
//            LOGGER.setLogLevel(RFLogger.Severity.INFO);
//            LOGGER.log("Intake.countPixels() : PIXEL COUNT CHANGED, count : "+pixelCount + " --> " + count);
//            pixelCount=count;
//        }
        return count;
    }


    /**
     * updates the state machine, log in general and intake surface
     * updates sensor information, triggers following action to reverse/stop intaking
     */
    public void update(){
        double power = this.getPower();
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("intake power:" + power);
        for(var i : IntakeStates.values()) {
            if(i.state)
                packet.put("IntakeState",i.name());
        }
        if(IntakeStates.STOPPED.state){
            stopIntake();
        }
        double pos = super.getCurrentPosition();
        packet.put("intakePos", pos);
        packet.put("intakeRevs", pos/HALF_TICKS_PER_REV);
        packet.put("intakeMod", pos%HALF_TICKS_PER_REV);

    }
}
