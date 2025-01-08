package org.firstinspires.ftc.teamcode.SystemsFSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Arm;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Claw;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;

import java.util.ArrayList;

public class Robot {

    private Drivetrain drivetrain;
    private Deposit deposit;
    private Intake intake;

    private GamepadEx controller;
    private Logger logger;

    public enum States {
        cycling,
        transferReady;
    }

    private States currentState;
    private Deposit.TargetState depositDesiredState;
    private Intake.SystemState intakeDesiredState;

    private boolean interference;

    private boolean clawSetForTransfer = false;


    public Robot(Hardware hardware, GamepadEx controller, Logger logger) {

        this.controller = controller;
        this.logger = logger;

        drivetrain = new Drivetrain(hardware, controller, logger, false);
        deposit = new Deposit(hardware, controller, logger);
        intake = new Intake(hardware, controller, logger);

        deposit.setTargetState(Deposit.TargetState.transfer);
        intake.setTargetState(Intake.SystemState.Stowed);

        findState();
    }

    public void update() {
        drivetrain.update();
        deposit.update();
        intake.update();
        findState();
    }

    public void command() {

        interferenceCheck();

        switch (currentState) {
            case cycling:

                // Need to prevent intake extension if the deposit is still at transfer position with sample
                if (controller.wasJustPressed(GamepadKeys.Button.B)) {

                    if (intake.getTargetSystemState() == Intake.SystemState.Deployed) {
                        intake.setTargetState(Intake.SystemState.Intaking);
                    } else {
                        intake.setTargetState(Intake.SystemState.Deployed);
                    }

                }

                if (controller.wasJustPressed(GamepadKeys.Button.X)) {
                    intake.setTargetState(Intake.SystemState.Stowed);
                }

                // If there is no interference potential, then all actions can be performed optimally for deposit
                if (!interference) {
                    deposit.setTargetState(depositDesiredState);
                } else {

                    if (depositDesiredState == Deposit.TargetState.specIntake) {
                        deposit.setTargetState(Deposit.TargetState.preSpecIntake);
                    } else {
                        deposit.setTargetState(Deposit.TargetState.preTransfer);
                    }


                }

                clawSetForTransfer = false;

                break;

            case transferReady:
                deposit.setTargetState(Deposit.TargetState.transfer);
                intake.setTargetState(Intake.SystemState.Stowed);


                // TODO: This logic sets the transfer behind by a loop if the claw is already open before transfer begins
                if (clawSetForTransfer) {
                    deposit.gripClaw();

                    if (deposit.getClawStatus() == Claw.Status.gripped) {
                        intake.hasSample = false;
                    }
                } else {
                    deposit.releaseClaw();
                    if (deposit.getClawStatus() == Claw.Status.released) {
                        clawSetForTransfer = true;
                    }

                }



        }

        drivetrain.command();
        deposit.command();
        intake.command();

    }

    public void log() {
        logger.log("<b>" + "--Robot--" + "</b>", "", Logger.LogLevels.production);

        logger.log("Current State", currentState, Logger.LogLevels.production);
        logger.log("Deposit Desired State", depositDesiredState, Logger.LogLevels.production);
        logger.log("Intake Desired State", intakeDesiredState, Logger.LogLevels.production);

        logger.log("Interference", interference, Logger.LogLevels.debug);

        drivetrain.log();
        deposit.log();
        intake.log();
    }

    public void setDepositDesiredState(Deposit.TargetState state) {
        depositDesiredState = state;
    }

    public void setIntakeDesiredState(Intake.SystemState state) {
        intakeDesiredState = state;
    }

    public Intake.SystemState getIntakeDesiredState() {
        return intakeDesiredState;
    }

    public void goToDeposit() {

        // If sample is yellow
        if (intake.getLastSeenColor() == SampleDetector.SampleColor.yellow) {
            setDepositDesiredState(Deposit.TargetState.sampleDeposit);
        }

        // If sample is alliance colored
        if ((intake.getLastSeenColor() == SampleDetector.SampleColor.blue) || (intake.getLastSeenColor() == SampleDetector.SampleColor.red)) {

            if (deposit.getTargetState() != Deposit.TargetState.specIntake) {
                setDepositDesiredState(Deposit.TargetState.specIntake);
            } else {
                setDepositDesiredState(Deposit.TargetState.specDepositReady);
            }

        }

    }

    public void clipSpec() {

        if (deposit.getTargetState() == Deposit.TargetState.specDepositReady) {
            setDepositDesiredState(Deposit.TargetState.specDepositClipped);
        } else if (deposit.getTargetState() == Deposit.TargetState.specDepositClipped) {
            setDepositDesiredState(Deposit.TargetState.specDepositReady);
        }

    }

    public void releaseClaw() {
        deposit.releaseClaw();
    }

    public void setAcceptedSamples(ArrayList<SampleDetector.SampleColor> colors) {
        intake.setAcceptableColors(colors);
    }

    public void switchColor() {
        if (intake.getLastSeenColor() == SampleDetector.SampleColor.yellow) {
            intake.setLastSeenColor(SampleDetector.SampleColor.blue);
        } else {
            intake.setLastSeenColor(SampleDetector.SampleColor.yellow);
        }
    }

    private void findState() {

        // If either the Intake or Deposit isnt at their transfer ready positions then the state is cycling
        if (intake.getCurrentSystemState() != Intake.SystemState.Stowed || deposit.getCurrentState() != Deposit.TargetState.transfer || !intake.hasSample) {
            currentState = States.cycling;

        } else { // If all of previous conditions are met, then we can assume we are ready to transfer
            currentState = States.transferReady;
        }

    }

    private void interferenceCheck() {

        boolean stowInterference = false;

        // If the intake is stowed, or is trying to stow, there is stow interference
        if (intake.getCurrentSystemState() == Intake.SystemState.Stowed || intake.getTargetSystemState() == Intake.SystemState.Stowed) {
            stowInterference = true;
        }

        // If there is stow interference, then check if the deposit is going to or from transfer
        // **Ignores potential for interference when going from spec intake, where you might be flipping the arm over too fast
        // For now, if there is stow interference and the slides are either below safe height, or trying to go below the safe height, we assume there is interference, but this is obviously not true, could be fixed by interference zones**
        if (stowInterference) {

            deposit.updateSlidesafe();

            if ((deposit.getSlideTargetCM() <= DepositConstants.slidePreTransferPos || deposit.getSlideCurrentCM() < DepositConstants.slidePreTransferPos - DepositConstants.slidePositionTolerance) && !(deposit.getSlidesDownSafe())) {

                interference = true;

            } else {
                interference = false;
            }

            if (depositDesiredState  == Deposit.TargetState.specIntake && deposit.getSlideCurrentCM() <= DepositConstants.slidePreTransferPos - DepositConstants.slidePositionTolerance && (deposit.arm.getRightServoEncPos() <= DepositConstants.armRightEncSlideDownSafePos - DepositConstants.armRightPositionTolerance) && deposit.arm.getRightSetPosition() > DepositConstants.armRightSampleDepositPos) {
                interference  = true;
            }



        }
    }
}
