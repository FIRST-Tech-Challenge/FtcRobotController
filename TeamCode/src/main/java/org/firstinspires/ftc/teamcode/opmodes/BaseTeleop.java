package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.alignment.AlignPID;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public abstract class BaseTeleop extends OpMode {

    //charlie
    protected Robot charlie;
    protected FieldConstants.Alliance alliance;

    //vision
    protected int[] tagsA, tagsB;
    private final double FIELD_CENTER_Y = 72.0;
    protected boolean wasVisionAiming = false;
    protected boolean wasOdoAiming = false;

    //pid
    protected AlignPID align;
    protected DrivePowers powers;

    //controls
    ToggleButtonReader polliButton, headingReset, aimingButton;

    @Override
    public void init() {
        charlie = new Robot(hardwareMap);

        charlie.shooter.closeGates();
        charlie.intake.setPolliUp();
        charlie.shooter.setIdleHood();

        polliButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER);
        headingReset = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        aimingButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_LEFT);

        align = new AlignPID(FieldConstants.hive(alliance, true));

        //update from teleop
        charlie.follower.setPose(FieldConstants.endAutoPose);
    }

    protected void handleAlignmentAndShooting() {
        boolean intentToShoot = gamepad1.right_trigger > 0.02 && gamepad1.left_trigger > 0.02;
        boolean aimingPressed = aimingButton.isDown() || intentToShoot;

        //1. odo rough alignment
        if (aimingPressed) {
            if (!wasOdoAiming) {
                //pick the point once, when the button goes down, so it can't flip mid-turn
                Pose p = charlie.follower.pose();
                align.setTarget(FieldConstants.closestHive(alliance, p.x(), p.y()));
                align.reset();
            }
            powers = align.aim(charlie.follower, powers);
        }

        //2. vision validation and apply fix
        if (Robot.visionEnabled) {
            if (intentToShoot && !wasVisionAiming) {
                align.reset();
            }

            if (intentToShoot) {
                Vision.TargetState state = charlie.vision.getTargetState(tagsA, tagsB);

                //determine which half of the field the robot is currently on
                boolean onSideA = charlie.follower.pose().y() > FIELD_CENTER_Y;

                //if we are on that side with the direction up
                boolean validTarget = false;
                if (onSideA && state == Vision.TargetState.A_UP_IN_RANGE) {
                    validTarget = true;
                } else if (!onSideA && state == Vision.TargetState.B_UP_IN_RANGE) {
                    validTarget = true;
                }

                if (validTarget) {

                    //apply the fix subclass of vision
                    Vision.Fix fix = charlie.vision.evaluateFix(charlie.follower.pose());
                    if (fix.apply) {
                        charlie.follower.setPose(fix.pose);
                    }

                    //align explicitly to the side we are on
                    powers = align.autoAim(charlie.follower, powers, alliance, onSideA);

                    charlie.shooter.openGates();

                } else {
                    //wrong side of field, out of range, or NONE (e.g. not valid tags detected)
                    charlie.shooter.closeGates();

                    if (!wasVisionAiming) {
                        gamepad1.rumble(1.0, 1.0, 500);
                    }
                }
            } else {
                charlie.shooter.closeGates();
            }

            wasVisionAiming = intentToShoot;
        }

        wasOdoAiming = aimingPressed;
    }

    @Override
    public void loop() {
        charlie.shooter.spinUp();
        charlie.shooter.shooterUpdate();

        //base translation
        DrivePowers robotRel = new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        powers = ManualDrive.fieldCentric(robotRel, charlie.follower.pose().heading(), -FieldConstants.driverForward(alliance));

        //sequential aiming and shooting handler
        handleAlignmentAndShooting();

        // relocalise button
        if (headingReset.wasJustPressed()) {
            charlie.follower.setPose(FieldConstants.RELOCALIZE_POSE);
        }

        Pose robotPose = charlie.follower.pose();

        headingReset.readValue();
        polliButton.readValue();
        aimingButton.readValue();

        //update drivetrain
        ManualDrive.driveOrHold(charlie.follower, powers);
        charlie.follower.update();

        //intake
        if (gamepad1.left_trigger    > 0.02) charlie.intake.startIntake();
        else charlie.intake.idleIntake();

        //transfer
        if (gamepad1.right_trigger > 0.2)  charlie.intake.startTransfer();
        else charlie.intake.stopTransfer();

        //polli
        if (polliButton.isDown()) {  charlie.intake.setPolliUp(); }
        else { charlie.intake.setPolliDown(); }

        //shooter
        if (!Robot.visionEnabled) {
            if (gamepad1.right_trigger > 0.02 && gamepad1.left_trigger > 0.02) charlie.shooter.openGates();
            else charlie.shooter.closeGates();
        }

        telemetry.addData("current velocity", charlie.shooter.getCurrentVelocity());
        telemetry.addData("target velocity", charlie.shooter.targetVelocity);
        telemetry.addData("error", charlie.shooter.shooterError);
        telemetry.addData("power", charlie.shooter.power);
        telemetry.addData("hood pos", charlie.shooter.getHoodPos());
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }
}