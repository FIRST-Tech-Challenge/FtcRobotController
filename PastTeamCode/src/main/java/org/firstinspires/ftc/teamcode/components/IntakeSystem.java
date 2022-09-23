
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helpers.TeamState;

/**
 * IntakeSystem.java is a component which
 * comprises of two enums in order to convey the state
 * of the system and be manipulated by state machines used in OpModes.
 * It has one motor, which can either be moving (ingesting) - or not.
 * Realize the motor's polarity is determined by set velocity's signum.
 * Understanding this should be pretty straight forward.
 */

// TODO - TEST AND FIND THIS NUMBER
public class IntakeSystem {
    // IntakeState
    private enum IntakeState {
        IDLE,
        TAKE_IN,
        SPIT_OUT,
        CAROUSEL
    }
    private IntakeState currentState;

    private static final double optimalSpinningSpeed = 0.75;
    // Hardware
    public final DcMotor motorLeft; //TODO
    public final DcMotor motorRight;

    /**
     * Creates the IntakeSystem Object
     * @param motorLeft to represent the motor which rotates in order to s'word the object in.
     */
    public IntakeSystem(DcMotor motorLeft, DcMotor motorRight) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        initMotors();
    }

    /**
     * Initializes the motors
     */
    public void initMotors() {
        currentState = IntakeState.IDLE;
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setPower(0);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setPower(0);
    }

    /**
     * Intakes rings
     */
    public void take_in() {
            currentState = IntakeState.TAKE_IN;
            motorLeft.setPower(-1);
            motorRight.setPower(1);
    }

    /**
     * Intakes rings
     */
    public void spit_out() {
            currentState = IntakeState.SPIT_OUT;
            motorLeft.setPower(0.31);
            motorRight.setPower(-0.31);
    }

    public void spit_out(double power) {
        currentState = IntakeState.SPIT_OUT;
        motorLeft.setPower(power);
        motorRight.setPower(-power);
    }

    /**
     * Intakes rings
     */
    public void Carousel(TeamState teamState) {
        if (currentState != IntakeState.CAROUSEL) {
            currentState = IntakeState.CAROUSEL;
            motorLeft.setPower((teamState == TeamState.RED ? -1 : 1) * optimalSpinningSpeed);
            motorRight.setPower((teamState == TeamState.RED ? -1 : 1) * optimalSpinningSpeed);
        }
    }

    public void setPower(double num) {
        motorLeft.setPower(num);
        motorRight.setPower(num);
    }



    public void setIdle(){
        setPower(0);
        currentState = IntakeState.IDLE;
    }

    /**
     * Shuts down the motor
     */
    public void stop() {
        if (currentState == IntakeState.TAKE_IN || currentState == IntakeState.SPIT_OUT) {
            currentState = IntakeState.IDLE;
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }


}