package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    // EJECT = full and an extra ball came in, so spit it back out (happens by itself)
    // SHOOT = open the door and run the rollers to push the balls into the flywheel
    public enum IntakeState { OFF, INTAKE, FULL, REVERSE, EJECT, SHOOT }

    // =====================================================================
    // SETTINGS - everything marked "TODO" below is a placeholder.
    //   TODO REPLACE = must change before the robot will run
    //   TODO TUNE    = works as a starting guess, adjust while testing
    // Tip: View > Tool Windows > TODO lists every one of these.
    // =====================================================================

    // ---- hardware names: must EXACTLY match the robot configuration on the Driver Hub ----
    private static final String LOWER_MOTOR_NAME = "lowerIntake";   // TODO REPLACE: lower roller motor name
    private static final String UPPER_MOTOR_NAME = "upperIntake";   // TODO REPLACE: upper roller motor name (delete if one motor drives both rollers)
    private static final String BEAM_BREAK_NAME  = "beamBreak";     // TODO REPLACE: beam-break sensor name
    private static final String DOOR_SERVO_NAME  = "door";          // TODO REPLACE: door servo name

    // ---- door servo positions (0.0 to 1.0) ----
    private static final double DOOR_OPEN   = 0.5;   // TODO REPLACE: position that lets balls into the flywheel
    private static final double DOOR_CLOSED = 0.0;   // TODO REPLACE: position that blocks the flywheel

    // ---- motor powers (-1.0 to 1.0) ----
    private static final double INTAKE_POWER  = 0.8;    // TODO TUNE: speed while collecting balls
    private static final double FULL_POWER    = 0.2;    // TODO TUNE: slow hold once full (too high = balls jam/squish)
    private static final double REVERSE_POWER = -0.8;   // TODO TUNE: speed when spitting everything out
    private static final double EJECT_POWER   = -0.8;   // TODO TUNE: lower roller backwards to spit out an extra ball
    private static final double SHOOT_POWER   = 1.0;    // TODO TUNE: lower it if balls hit the flywheel too fast to shoot well

    // ---- counting / timing ----
    private static final int MAX_BALLS = 4;                     // TODO REPLACE: how many balls the robot can hold
    private static final double COUNT_COOLDOWN_SECONDS = 0.2;   // TODO TUNE: raise if one ball counts twice (holes in the balls)
    private static final double EJECT_TIME_SECONDS = 0.5;       // TODO TUNE: long enough for the extra ball to fully leave

    // ---- hardware ----
    private final DcMotorEx lowerRoller;
    private final DcMotorEx upperRoller;   // TODO REPLACE: delete this (and every upperRoller line) if one motor drives both rollers
    private final DigitalChannel beamBreak;
    private final Servo door;

    // ---- state ----
    private IntakeState state = IntakeState.OFF;
    private int ballCount = 0;
    private boolean ballWasInBeam = false;
    private final ElapsedTime countTimer = new ElapsedTime();
    private final ElapsedTime ejectTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
        lowerRoller = hardwareMap.get(DcMotorEx.class, LOWER_MOTOR_NAME);
        upperRoller = hardwareMap.get(DcMotorEx.class, UPPER_MOTOR_NAME);
        beamBreak = hardwareMap.get(DigitalChannel.class, BEAM_BREAK_NAME);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        door = hardwareMap.get(Servo.class, DOOR_SERVO_NAME);
        door.setPosition(DOOR_CLOSED);

        lowerRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO TEST on the robot: run INTAKE and watch the rollers. Both must push the
        // ball INWARD. If one spins the wrong way, flip its direction here.
        lowerRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        upperRoller.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // ---- controls (call these from your OpMode, e.g. when a button is pressed) ----
    public void setState(IntakeState newState) {
        // shooting empties the intake, so start counting from 0 again
        if (newState == IntakeState.SHOOT) {
            ballCount = 0;
        }
        this.state = newState;
    }

    // shortcut: push the balls into the flywheel (spin the flywheel up first!)
    public void shoot() {
        setState(IntakeState.SHOOT);
    }

    public void resetBallCount() {
        this.ballCount = 0;
    }

    // ---- getters for telemetry and other mechanisms ----
    public IntakeState getState() { return state; }
    public int getBallCount() { return ballCount; }
    public boolean isFull() { return ballCount >= MAX_BALLS; }
    public double getThrottle() { return lowerRoller.getPower(); }
    public String getSensorDebug() { return "beam raw=" + beamBreak.getState() + " broken=" + isBeamBroken(); }

    // Most beam-break sensors read LOW (false) when something is blocking the beam.
    // TODO TEST on the robot: show getSensorDebug() in telemetry. "broken" should be true
    // ONLY when a ball is in the beam. If it's backwards, remove the "!" below.
    private boolean isBeamBroken() {
        return !beamBreak.getState();
    }

    // ---- call this once every loop from your OpMode ----
    public void periodic() {

        // 1. did a ball JUST arrive at the sensor? (wasn't there last loop, is there now)
        boolean ballInBeam = isBeamBroken();
        boolean ballArrived = ballInBeam
                && !ballWasInBeam
                && countTimer.seconds() > COUNT_COOLDOWN_SECONDS;
        ballWasInBeam = ballInBeam;

        if (ballArrived && state == IntakeState.INTAKE && ballCount < MAX_BALLS) {
            // room for it: count it
            ballCount = ballCount + 1;
            countTimer.reset();
        } else if (ballArrived && state == IntakeState.FULL) {
            // already have 4: this is an extra ball, spit it out
            state = IntakeState.EJECT;
            ejectTimer.reset();
            countTimer.reset();
        }

        // done spitting it out? go back to holding the 4 we have
        if (state == IntakeState.EJECT && ejectTimer.seconds() > EJECT_TIME_SECONDS) {
            state = IntakeState.FULL;
        }

        // 2. switch automatically between taking balls in and being full
        if (state == IntakeState.INTAKE && ballCount >= MAX_BALLS) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.FULL && ballCount < MAX_BALLS) {
            state = IntakeState.INTAKE;
        }

        // 3. set the motors to match the state
        double lowerPower;
        double upperPower;
        switch (state) {
            case INTAKE:  lowerPower = INTAKE_POWER;  upperPower = INTAKE_POWER;  break;
            case FULL:    lowerPower = FULL_POWER;    upperPower = FULL_POWER;    break;
            case REVERSE: lowerPower = REVERSE_POWER; upperPower = REVERSE_POWER; break;
            // only the first roller spits out the extra ball;
            // the upper roller keeps holding the 4 we want to keep
            case EJECT:   lowerPower = EJECT_POWER;   upperPower = FULL_POWER;    break;
            case SHOOT:   lowerPower = SHOOT_POWER;   upperPower = SHOOT_POWER;   break;
            default:      lowerPower = 0.0;           upperPower = 0.0;           break;   // OFF
        }
        lowerRoller.setPower(lowerPower);
        upperRoller.setPower(upperPower);

        // door is only open while shooting; closed it keeps balls away from the flywheel
        door.setPosition(state == IntakeState.SHOOT ? DOOR_OPEN : DOOR_CLOSED);
    }
}
