package org.firstinspires.ftc.teamcode.SystemsFSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
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
        Intaking,
        waiting;
    }

    private ArrayList<SampleDetector.SampleColor> acceptableColors =  new ArrayList<>();

    private SystemState targetSystemState = SystemState.waiting;

    private double feedRate = 0.00;
    private double fedPosition = 0.00;
    private double maxFedPosition = IntakeConstants.maxExtensionPosition - IntakeConstants.readyPosition;
    private double minFedPosition = IntakeConstants.minIntakePosition - IntakeConstants.readyPosition;

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
        detector.update();
        slides.update();

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

                if (detector.getStatus() == SampleDetector.Status.sampleDetected) {
                    bucket.setRollerPower(IntakeConstants.stallingPower);
                    bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                } else {
                    bucket.setRollerPower(0.00);
                    bucket.setGatePosition(IntakeConstants.gateOpenPosition);
                }

                // Need to figure out how to command the gate position
                break;

            case Deployed:

                feed();
                bucket.setBucketPosition(IntakeConstants.bucketUpPosition);
                bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                slides.setTargetCM(IntakeConstants.readyPosition + fedPosition);
                bucket.setRollerPower(0.00);

                break;

            case Intaking:

                feed();
                if (slides.getPosition() >= IntakeConstants.minIntakePosition) {
                    bucket.setBucketPosition(IntakeConstants.bucketDownPosition);
                }
                if (bucket.getStatus() == Bucket.Status.down) {
                    bucket.setRollerPower(IntakeConstants.intakingPower);
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
                    } else {
                        bucket.setGatePosition(IntakeConstants.gateOpenPosition);
                    }


                } else {
                    bucket.setGatePosition(IntakeConstants.gateBlockedPosition);
                }
                // Need to add command for gate position based on sample detection

                break;


        }

        bucket.command();
        slides.command();
    }

    public void log(){
        logger.log("-Intake-", "", Logger.LogLevels.production);

        logger.log("Target System State", targetSystemState, Logger.LogLevels.production);

        logger.log("Accepetable Colors", acceptableColors.toString(), Logger.LogLevels.debug);

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

    private void feed() {
        feedRate = IntakeConstants.maxFeedRate * -controller.getRightY();
        fedPosition += feedRate * ( recordedTime / 1000.0 );
        fedPosition = Math.min(Math.max(fedPosition, minFedPosition), maxFedPosition);
    }


}

