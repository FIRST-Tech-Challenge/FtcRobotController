package org.firstinspires.ftc.teamcode.org.rustlib.vision;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Vector2d;

import java.util.function.Supplier;

public class CameraCameraActivationBox implements CameraActivationZone {
    private final Vector2d center;
    private final double width;
    private final double height;
    private final Rotation2d headingMin;
    private final Rotation2d headingMax;
    private final Supplier<Pose2d> poseSupplier;

    public CameraCameraActivationBox(Vector2d center, double width, double height, Rotation2d headingMin, Rotation2d headingMax, Supplier<Pose2d> poseSupplier) {
        if (width < 0) {
            throw new IllegalArgumentException("Width must be greater than or equal to zero.");
        } else if (height < 0) {
            throw new IllegalArgumentException("Height must be greater than or equal to zero.");
        }
        checkForIllegalHeadings(headingMin, headingMax);
        this.center = center;
        this.width = width;
        this.height = height;
        this.headingMin = headingMin;
        this.headingMax = headingMax;
        this.poseSupplier = poseSupplier;
    }

    public CameraCameraActivationBox(Vector2d center, double width, double height, Supplier<Pose2d> poseSupplier) {
        this(center, width, height, new Rotation2d(), Rotation2d.fromDegrees(359.9999), poseSupplier);
    }

    public CameraCameraActivationBox(Vector2d topLeft, Vector2d topRight, Vector2d bottomRight, Vector2d bottomLeft, Rotation2d headingMin, Rotation2d headingMax, Supplier<Pose2d> poseSupplier) {
        if (!isRectangle(topLeft, topRight, bottomLeft, bottomRight)) {
            throw new IllegalArgumentException("The provided points do not form a valid rectangle.");
        } else if (topLeft.y <= bottomLeft.y || topLeft.y <= bottomRight.y || topLeft.x >= topRight.x || topLeft.x >= bottomRight.x) {
            throw new IllegalArgumentException("Points must be provided in the following order: topLeft, topRight, bottomRight, bottomLeft.");
        }
        checkForIllegalHeadings(headingMin, headingMax);
        this.center = new Vector2d((topLeft.x + topRight.x) / 2, (topLeft.y + bottomLeft.y) / 2);
        this.width = topRight.x - topLeft.x;
        this.height = topLeft.y - bottomLeft.y;
        this.headingMin = headingMin;
        this.headingMax = headingMax;
        this.poseSupplier = poseSupplier;
    }

    public CameraCameraActivationBox(Vector2d topLeft, Vector2d topRight, Vector2d bottomRight, Vector2d bottomLeft, Supplier<Pose2d> poseSupplier) {
        this(topLeft, topRight, bottomRight, bottomLeft, new Rotation2d(), Rotation2d.fromDegrees(359.9999), poseSupplier);
    }

    private boolean isRectangle(Vector2d topLeft, Vector2d topRight, Vector2d bottomRight, Vector2d bottomLeft) {
        return topLeft.y == topRight.y && bottomLeft.y == bottomRight.y && topLeft.x == bottomLeft.x && topRight.x == bottomRight.x;
    }

    private void checkForIllegalHeadings(Rotation2d headingMin, Rotation2d headingMax) {
        if (Rotation2d.unsigned_0_to_2PI(headingMin.getAngleRadians()) > Rotation2d.unsigned_0_to_2PI(headingMax.getAngleRadians())) {
            throw new IllegalArgumentException("The minimum heading must be less than the maximum heading.");
        }
    }

    @Override
    public boolean withinZone() {
        Pose2d botPose = poseSupplier.get();
        double heading = Rotation2d.unsigned_0_to_2PI(botPose.rotation.getAngleRadians());
        double halfWidth = width / 2;
        double halfHeight = height / 2;
        return botPose.x > center.x - halfWidth && botPose.x < center.x + halfWidth && botPose.y > center.y - halfHeight && botPose.y < center.y + halfHeight
                && heading > Rotation2d.unsigned_0_to_2PI(headingMin.getAngleRadians()) && heading < Rotation2d.unsigned_0_to_2PI(headingMax.getAngleRadians());
    }
}
