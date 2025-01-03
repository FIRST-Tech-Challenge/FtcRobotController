package org.firstinspires.ftc.teamcode.SystemsFSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.Hardware.Util.PosChecker;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Bucket;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.IntakeSlides;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Intake {
    private Logger logger;

    private IntakeSlides slides;
    private Bucket bucket;
    private SampleDetector detector;
    private GamepadEx controller;

    public enum SystemState {
        Stowed,
        Deployed,
        Intaking;
    }

    private ArrayList<SampleDetector.SampleColor> acceptableColors =  new ArrayList<>();

    private SystemState targetSystemState;
    private SystemState currentSystemState;

    private double feedRate = 0.00;
    private double fedPosition = 0.00;
    private double maxFedPosition = IntakeConstants.maxExtensionPosition - IntakeConstants.readyPosition;
    private double minFedPosition = IntakeConstants.minIntakePosition - IntakeConstants.readyPosition;

    public boolean hasSample = false;

    private SampleDetector.SampleColor lastSeenColor;

    private Timing.Timer timer = new Timing.Timer(999999, TimeUnit.MILLISECONDS);
    private double recordedTime = 0.00;

    public Intake(Hardware hardware, GamepadEx controller, Logger logger) {
        this.controller = controller;
        this.logger = logger;
        slides = new IntakeSlides(hardware, this.logger);
        bucket = new Bucket(hardware, this.logger);
        detector = new SampleDetector(hardware, this.logger);
    }

    public void update() {
        bucket.update();
        slides.update();

        findsState();

        // Only if intaking then update the detector
        if (currentSystemState == SystemState.Intaking) {
            detector.update();
        }

        recordedTime = timer.elapsedTime();
        timer.start();
    }


    public void command() {

        switch (targetSystemState)  {
            case Stowed:
                fedPosition = 0.00;

                bucket.setBucketPosition(IntakeConstants.bucketUpPosition);
                if (bucket.getStatus() == Bucket.Status.up) {
                    slides.setTargetCM(IntakeConstants.stowedPosition);
                }

                if (hasSample) {
                    bucket.setRollerPower(IntakeConstants.stallingPower);
                    bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                } else {
                    bucket.setRollerPower(0.00);
                    bucket.setGatePosition(IntakeConstants.gateOpenPosition);
                }

                break;

            case Deployed:

                feed();
                hasSample = false;
                detector.clearDistanceBuffer();
                bucket.setBucketPosition(IntakeConstants.bucketUpPosition);
                bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                slides.setTargetCM(IntakeConstants.readyPosition + fedPosition);

                // Intake Reversing
                if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.1) {
                    bucket.setRollerPower(-1);
                } else {
                    bucket.setRollerPower(0);
                }

                break;

            case Intaking:

                feed();
                if (slides.getPosition() >= IntakeConstants.minIntakePosition) {
                    bucket.setBucketPosition(IntakeConstants.bucketDownPosition);
                }
                if (bucket.getStatus() == Bucket.Status.down) {
                    // Intake Reversing
                    if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.1) {
                        bucket.setRollerPower(-1);
                    } else {
                        bucket.setRollerPower(IntakeConstants.intakingPower);
                    }

                } else {
                    bucket.setRollerPower(0.00);
                }
                slides.setTargetCM(IntakeConstants.readyPosition + fedPosition);

                if (detector.getStatus() == SampleDetector.Status.sampleDetected) {

                    boolean acceptedSample = false;

                    for (SampleDetector.SampleColor color : acceptableColors) {
                        if (color == detector.getSampleColor()) {
                            acceptedSample = true;
                            break;
                        }
                    }

                    if (acceptedSample) {
                        targetSystemState = SystemState.Stowed;
                        lastSeenColor = detector.getSampleColor();
                        hasSample = true;
                    } else if (detector.getSampleColor() == SampleDetector.SampleColor.unknown) {
                        bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                    } else {
                        bucket.setGatePosition(IntakeConstants.gateOpenPosition);
                    }


                } else {
                    bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                }

                break;


        }

        bucket.command();
        slides.command();
    }

    public void log(){
        logger.log("<b>" + "-Intake-" + "</b>", "", Logger.LogLevels.production);

        logger.log("Target System State", targetSystemState, Logger.LogLevels.production);
        logger.log("Current State", currentSystemState, Logger.LogLevels.production);
        logger.log("Last seen color", lastSeenColor, Logger.LogLevels.production);

        logger.log("Accepetable Colors", acceptableColors.toString(), Logger.LogLevels.debug);
        logger.log("Has Sample", hasSample, Logger.LogLevels.debug);

        logger.log("Feedrate", feedRate, Logger.LogLevels.developer);
        logger.log("Fed Position", fedPosition, Logger.LogLevels.developer);
        logger.log("Recorded Time", recordedTime, Logger.LogLevels.developer);

        bucket.log();
        detector.log();
        slides.log();
    }

    public void setTargetState(SystemState state) {
        targetSystemState = state;
    }

    public SystemState getTargetSystemState(){
        return targetSystemState;
    }

    public void setAcceptableColors(ArrayList<SampleDetector.SampleColor> colors) {
        acceptableColors = colors;
    }

    public SystemState getCurrentSystemState() {
        return currentSystemState;
    }

    public SampleDetector.SampleColor getLastSeenColor() {
        return lastSeenColor;
    }

    public void setLastSeenColor(SampleDetector.SampleColor color) {
        lastSeenColor = color;
    }

    private void feed() {

        // Only if the Y value of the joystick is above 0.3 feed
        if (Math.abs(controller.getRightY()) >= 0.3) {
            feedRate = IntakeConstants.maxFeedRate * -controller.getRightY();
            fedPosition += feedRate * ( recordedTime / 1000.0 );
            fedPosition = Math.min(Math.max(fedPosition, minFedPosition), maxFedPosition);
        }
    }

    private void findsState() {

        if ((slides.getPosition() <= (0.0 + IntakeConstants.intakeSlidePositionTolerance)) && targetSystemState == SystemState.Stowed && slides.getVelocity() <= 1) {
            currentSystemState = SystemState.Stowed;
        } else if (bucket.getStatus() == Bucket.Status.up) {
            currentSystemState = SystemState.Deployed;
        } else {
            currentSystemState = SystemState.Intaking;
        }

    }


}

