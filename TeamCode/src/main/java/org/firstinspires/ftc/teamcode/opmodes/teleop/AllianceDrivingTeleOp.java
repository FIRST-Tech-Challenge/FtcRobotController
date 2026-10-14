package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Controls;

import java.util.function.DoubleSupplier;

public abstract class AllianceDrivingTeleOp extends OpMode {
    private final String allianceName;
    private final DoubleSupplier basket1X;
    private final DoubleSupplier basket1Y;
    private final DoubleSupplier basket2X;
    private final DoubleSupplier basket2Y;
    private final KronBot robot = new KronBot();
    private Controls controls;
    private Limelight3A limelight;
    private Pose selectedStartPose;
    private int pipelineNumber = Constants.LIMELIGHT_DEFAULT_PIPELINE;

    protected AllianceDrivingTeleOp(
            String allianceName,
            DoubleSupplier basket1X,
            DoubleSupplier basket1Y,
            DoubleSupplier basket2X,
            DoubleSupplier basket2Y) {
        this.allianceName = allianceName;
        this.basket1X = basket1X;
        this.basket1Y = basket1Y;
        this.basket2X = basket2X;
        this.basket2Y = basket2Y;
    }

    @Override public void init() {
        controls = new Controls(gamepad1);
        selectedStartPose = leftStartPose();
        robot.initialize(hardwareMap, selectedStartPose);
        robot.drive.configureAimTargets(basket1X, basket1Y, basket2X, basket2Y);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineNumber);
        addInitTelemetry();
    }

    @Override public void init_loop() {
        controls.update();
        if (controls.dpadLeft.justPressed()) {
            selectedStartPose = leftStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadRight.justPressed()) {
            selectedStartPose = rightStartPose();
            robot.setPose(selectedStartPose);
        }

        robot.update();
        addInitTelemetry();
    }

    @Override public void start() {
        limelight.start();
        robot.drive.startManual();
    }

    @Override public void loop() {
        controls.update();

        if (controls.cross.justPressed()
                && !robot.drive.isFieldCentric()
                && !robot.drive.isAiming()) {
            robot.drive.driveTo(centerPose());
        }
        if (controls.circle.justPressed()) robot.drive.startManual();
        if (controls.triangle.justPressed() && robot.drive.isManual()) {
            robot.drive.toggleFieldCentric();
        }
        if (controls.leftTrigger.getValue() > 0.5 && robot.drive.isManual()) {
            robot.drive.startAiming();
//            if (robot.drive.isAiming()) robot.drive.stopAiming();
//            else robot.drive.startAiming();
        }
        else {
            robot.drive.stopAiming();
        }
        if (controls.leftStickButton.justPressed()) {
            robot.setPose(robot.follower.getPose().getX() < 72.0
                    ? leftStartPose() : rightStartPose());
        }

        if (controls.dpadUp.justPressed()) {
            selectPipeline(Constants.LIMELIGHT_DEFAULT_PIPELINE);
        } else if (controls.dpadDown.justPressed()) {
            selectPipeline(Constants.LIMELIGHT_ALTERNATE_PIPELINE);
        }

        if (robot.intake != null) {
            if (controls.rightBumper.justPressed()) robot.intake.toggle(false);
            if (controls.leftBumper.justPressed()) robot.intake.toggle(true);
        }

        robot.drive.runManual(controls);
        robot.update();
        addDriverTelemetry();
        telemetry.update();
    }

    @Override public void stop() {
        try {
            robot.release();
        } finally {
            if (limelight != null) limelight.stop();
        }
    }

    private void selectPipeline(int pipeline) {
        if (pipeline == pipelineNumber) return;
        pipelineNumber = pipeline;
        limelight.pipelineSwitch(pipelineNumber);
    }

    private void addInitTelemetry() {
        telemetry.addData("Alliance", allianceName);
        telemetry.addData("Status", "Initialized without NextFTC");
        telemetry.addData("Start pose", "(%.1f, %.1f, %.0f deg)",
                selectedStartPose.getX(), selectedStartPose.getY(),
                Math.toDegrees(selectedStartPose.getHeading()));
        telemetry.addLine("INIT: D-pad left/right selects the physical starting side");
        robot.drive.telemetry(telemetry);
        if (!robot.drive.isSelectedAimTargetConfigured()) {
            telemetry.addLine("WARNING: enter this alliance's basket coordinates in Constants");
        }
        telemetry.update();
    }

    private void addDriverTelemetry() {
        telemetry.addData("Alliance", allianceName);
        robot.drive.telemetry(telemetry);
        if (robot.intake != null) robot.intake.telemetry(telemetry);
        addVisionTelemetry();
        telemetry.addLine("X: center | O: manual | Triangle: field centric");
        telemetry.addLine("Square: autoaim | R3: basket 1/2 | L3: reset pose");
        telemetry.addLine("D-pad up/down: Limelight pipeline 3/4");
    }

    private void addVisionTelemetry() {
        telemetry.addData("Limelight pipeline", pipelineNumber);
        LLResult result = limelight.getLatestResult();
        boolean valid = result != null && result.isValid();
        telemetry.addData("Vision target valid", valid);
        if (!valid) return;
        telemetry.addData("tx", "%.2f", result.getTx());
        telemetry.addData("ty", "%.2f", result.getTy());
        Pose3D botpose = result.getBotpose();
        if (botpose != null) telemetry.addData("Limelight botpose", botpose);
    }

    private static Pose leftStartPose() {
        return new Pose(Constants.LEFT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose rightStartPose() {
        return new Pose(Constants.RIGHT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose centerPose() {
        return new Pose(Constants.CENTER_X, Constants.CENTER_Y,
                Math.toRadians(Constants.CENTER_HEADING_DEG));
    }
}
