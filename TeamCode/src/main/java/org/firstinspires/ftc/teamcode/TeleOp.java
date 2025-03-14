package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOp extends LinearOpMode {
    // Initialize subsystems
    private ArmControl arm;
    private DriveControl myDrive;
    private clawcontrol claw;

    // Define states for the arm movement
    private enum ARM_STATE {
        NONE,
        RESTING,
        ROBOT_HANG,
        PICK_SPECIMEN,
        PICK_SAMPLE,
        SNAP_SPECIMEN,
        LOW_BASKET,
        ARM_VERTICAL,
        ENTER_EXIT_SUB,
        HANG_SPECIMEN,
        HIGH_BASKET,
    }
    private ARM_STATE armState = ARM_STATE.NONE;

    // Timekeeper for gamepad feedback
    private ElapsedTime timeKeeper = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        myDrive = new DriveControl(this.hardwareMap);
        claw = new clawcontrol(this.hardwareMap);
        arm = new ArmControl(this.hardwareMap);

        waitForStart(); // Wait for the start button

        while (opModeIsActive()) {
            // Move drivetrain with gamepad1 (always runs)
            myDrive.driveWithGamepad(gamepad1);

            // Handle state changes based on button presses
            detectStateChange();

            // Process the current state
            processState();

            // processes the bumpers
            processBumpers();

            // Telemetry feedback
            telemetry.addData("Arm State", armState);
            telemetry.addData("Arm Busy", armIsBusy());
            telemetry.update();
        }
    }

    private void detectStateChange() {
        if (gamepad1.dpad_left && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.HIGH_BASKET;
            gamepadFeedback();
            timeKeeper.reset();
        } else if (gamepad1.dpad_right && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.LOW_BASKET;
            gamepadFeedback();
            timeKeeper.reset();
        } else if (gamepad1.dpad_up && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.ARM_VERTICAL;
            gamepadFeedback();
            timeKeeper.reset();
        } else if (gamepad1.dpad_down && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.RESTING;
            gamepadFeedback();
            timeKeeper.reset();
        } else if (gamepad1.a && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.ENTER_EXIT_SUB;
            gamepadFeedback();
            timeKeeper.reset();
        } else if (gamepad1.b && timeKeeper.seconds() >= 0.5) {
            armState = ARM_STATE.PICK_SAMPLE;
            gamepadFeedback();
            timeKeeper.reset();
        }
    }

    private void processState() {
        switch (armState) {
            case HIGH_BASKET:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> setArmTarget(Constants.ArmPosHighBasket, Constants.SlidePosHighBasket,
                                        Constants.ArmVelocityHighBasket, Constants.SlideVelocityHighBasket)),
                                new InstantCommand(() -> claw.clawPosScoreHighBasket(Constants.ClawPosLeftBasket, Constants.ClawPosRightBasket)),
                                new InstantCommand(() -> claw.openClaw())
                        )
                );
                break;

            case LOW_BASKET:
                setArmTarget(Constants.ArmPosLowBasket, Constants.SlidePosLowBasket,
                        Constants.ArmVelocityLowBasket, Constants.SlideVelocityLowBasket);
                // move claw to score high basket
                claw.clawPosScoreHighBasket(Constants.ClawPosLeftBasket, Constants.ClawPosRightBasket);
                break;

            case RESTING:
                setArmTarget(Constants.ArmPosLowChamber, Constants.SlidePosLowChamber,
                        Constants.ArmVelocityLowChamber, Constants.SlideVelocityLowChamber);
                break;

            case ENTER_EXIT_SUB:
                setArmTarget(Constants.ArmPosEnterSub, 0, Constants.ArmVelocityEnterSub, 0);
                break;

            case PICK_SAMPLE:
                setArmTarget(Constants.ArmPosIntake, 0, Constants.ArmVelocityIntake, 0);
                break;
        }

        // Once the arm has completed its motion, return to NONE
        if (!armIsBusy()) {
            armState = ARM_STATE.NONE;
        }
    }

    private void processBumpers() {
        if (gamepad1.right_bumper) {
            claw.openClaw();
            gamepadFeedback();
        }
        if (gamepad1.left_bumper) {
            claw.closeClaw();
            gamepadFeedback();
        }
    }

    private void gamepadFeedback() {
        // Implement visual/auditory feedback here
        telemetry.addLine("Button Pressed");
        telemetry.update();
    }

    private void setArmTarget(int armPos, int slidePos, double armVelocity, double slideVelocity) {
        arm.moveArmAndSlide(armPos, slidePos, armVelocity, slideVelocity);
    }

    private boolean armIsBusy() {
        return arm.isBusy() || arm.isSlideBusy(); // Ensure these methods exist in ArmControl
    }
}
