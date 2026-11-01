package org.firstinspires.ftc.teamcode.follower.hardware;

import org.firstinspires.ftc.teamcode.follower.localization.Localizer;
import org.firstinspires.ftc.teamcode.follower.localization.MotionState;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PinpointLocalizer implements Localizer {

    public enum ResetMode {
        RECALIBRATE_IMU,
        RESET_AND_RECALIBRATE_IMU,
        NONE
    }

    private final GoBildaPinpointDriver pinpoint;
    private final DistanceUnit globalDistanceUnit;

    private MotionState motionState;
    private final ResetMode resetMode;

    public PinpointLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        this.globalDistanceUnit = config.globalDistanceUnit.get();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());

        pinpoint.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());

        if (config.ticksPerUnit.get().isPresent()) {
            pinpoint.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(), config.encoderResolutionUnit.get());
        } else {
            pinpoint.setEncoderResolution(config.podType.get());
        }

        pinpoint.setEncoderDirections(
                config.xPodDirection.get(),
                config.yPodDirection.get()
        );

        resetMode = config.resetMode.get();

        reset();
        update();
    }

    public void setPose(Pose pose) {
        pinpoint.setPosition(
                new Pose2D(
                        globalDistanceUnit,
                        pose.x(),
                        pose.y(),
                        AngleUnit.RADIANS,
                        pose.heading()
                )
        );

        if (motionState != null) {
            motionState = motionState.withPose(pose);
        } else {
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
        }
    }

    @Override
    public void update() {
        pinpoint.update();

        Pose pose = new Pose(
                pinpoint.getPosX(globalDistanceUnit),
                pinpoint.getPosY(globalDistanceUnit),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        Velocity velocity = new Velocity(
                pinpoint.getVelX(globalDistanceUnit),
                pinpoint.getVelY(globalDistanceUnit),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );

        motionState = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    public void reset() {
        if (resetMode == ResetMode.RESET_AND_RECALIBRATE_IMU) {
            pinpoint.resetPosAndIMU();
        } else if (resetMode == ResetMode.RECALIBRATE_IMU) {
            pinpoint.recalibrateIMU();
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}