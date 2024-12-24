package org.firstinspires.ftc.teamcode.SystemsFSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.Hardware.Util.PosChecker;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Arm;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Claw;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.DepositSlides;

public class Deposit {
    private Claw claw;
    private Arm arm;
    private DepositSlides slides;
    private Logger logger;
    private GamepadEx controller;

    public enum TargetState {
        transfer,
        preTransfer,
        specIntake,
        specDepositReady,
        specDepositClipped,
        sampleDeposit;
    }

    private TargetState targetState;

    public Deposit(Hardware hardware, GamepadEx controller, Logger logger){
        this.logger = logger;
        this.controller = controller;

        claw = new Claw(hardware, logger);
        arm = new Arm(hardware, logger);
        slides = new DepositSlides(hardware, logger);

    }

    public void update() {
        claw.update();
        arm.update();
        slides.update();
    }

    public void command() {

        switch (targetState) {
            case transfer:

                arm.setPosition(DepositConstants.armRightTransferPos);
                slides.setTargetCM(DepositConstants.slideTransferPos);

                break;

            case preTransfer:

                slides.setTargetCM(DepositConstants.slidePreTransferPos);

                // Checking to see if the slides are at or above the pre transfer position, with tolerance
                if ((slides.getPosition() - DepositConstants.slidePositionTolerance) >= DepositConstants.slideTransferPos) {
                    arm.setPosition(DepositConstants.armRightTransferPos);
                }

                break;

            case specIntake:

                // If arm isnt at the Intake Position then go to the pre transfer pos so that the arm can swing over
                if (arm.getStatus() != Arm.Status.SpecIntakePos) {
                    slides.setTargetCM(DepositConstants.slidePreTransferPos);
                } else {
                    slides.setTargetCM(DepositConstants.slideSpecIntakePos);
                }

                // If slides are at the pre transfer position, or going towards spec intake pos, the arm should be going to Spec Intake Position
                if (PosChecker.atLinearPos(slides.getPosition(), DepositConstants.slidePreTransferPos, DepositConstants.slidePositionTolerance) || slides.getTargetCM() == DepositConstants.slideSpecIntakePos) {
                    arm.setPosition(DepositConstants.armRightSpecIntakePos);
                } else {
                    arm.setPosition(DepositConstants.armRightTransferPos);
                }

                break;

            case specDepositReady:

                slides.setTargetCM(DepositConstants.slideSpecDepositReadyPos);

                // If slides are above transfer ready pos then move arm to the spec deposit ready position
                if ((slides.getPosition() - DepositConstants.slidePositionTolerance) >= DepositConstants.slideTransferPos) {
                    arm.setPosition(DepositConstants.armRightSpecDepositPos);
                } else {
                    arm.setPosition(DepositConstants.armRightSampleDepositPos);
                }

                break;

            case specDepositClipped:

                slides.setTargetCM(DepositConstants.slideSpecClippedPos);

                // If slides are above transfer ready pos then move arm to the spec clipped position

                if ((slides.getPosition() - DepositConstants.slidePositionTolerance) >= DepositConstants.slideTransferPos) {
                    arm.setPosition(DepositConstants.armRightSpecDepositPos);
                } else {
                    arm.setPosition(DepositConstants.armRightSampleDepositPos);
                }

                break;

            case sampleDeposit:

                slides.setTargetCM(DepositConstants.slideSampleDepositPos);

                // If above slide pre transfer pos then move arm to sample deposit, otherwise keep arm position as is
                if ((slides.getPosition() - DepositConstants.slidePositionTolerance) >= DepositConstants.slideTransferPos) {
                    arm.setPosition(DepositConstants.armRightSampleDepositPos);
                } else {
                    arm.setPosition(arm.getRightSetPosition());
                }

                break;

        }

        if (controller.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

            if (claw.getStatus() == Claw.Status.released) {
                claw.setTargetPosition(DepositConstants.clawClosedPos);
            } else if (claw.getStatus() == Claw.Status.gripped) {
                claw.setTargetPosition(DepositConstants.clawOpenPos);
            }

        }



        claw.command();
        arm.command();
        slides.command();
    }

    public void log() {
        logger.log("-Deposit-", "", Logger.LogLevels.production);

        logger.log("Target State", targetState, Logger.LogLevels.production);

        claw.log();
        arm.log();
        slides.log();
    }

    public void setTargetState(TargetState state) {
        targetState = state;
    }

    public TargetState getTargetState() {
        return targetState;
    }

}
