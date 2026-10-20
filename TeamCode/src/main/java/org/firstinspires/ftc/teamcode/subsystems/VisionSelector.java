package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

/**
 * Picks which camera sees the AprilTags, based on what is in the robot configuration.
 *
 * <p>A Limelight named {@code cfgLimelight} wins: it finds the tags on its own and Decode's
 * robot has one. Without it, a webcam named {@code cfgWebcam}. Without either, a
 * {@link Vision_None} that never sees a tag, so HyDrive still runs. Same idea as
 * {@link ImuSelector}.
 */
public final class VisionSelector {
    private VisionSelector() {
    }

    public static Vision Pick(HydraOpMode op, BotProperties properties) {
        if (op.mHardwareMap.tryGet(Limelight3A.class, properties.cfgLimelight) != null) {
            op.mTelemetry.addLine("Camera: Limelight 3A");
            return new Vision_Limelight(op, properties);
        }
        if (op.mHardwareMap.tryGet(WebcamName.class, properties.cfgWebcam) != null) {
            op.mTelemetry.addLine("Camera: webcam '" + properties.cfgWebcam + "'");
            return new Vision_Webcam(op, properties);
        }
        op.mTelemetry.addLine("Camera: none found. Aim assist is off.");
        return new Vision_None(op, properties);
    }
}
