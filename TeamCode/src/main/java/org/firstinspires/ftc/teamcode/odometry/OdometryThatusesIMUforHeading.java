package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.GivesPosition;
import org.firstinspires.ftc.teamcode.imu.IMU;
import org.firstinspires.ftc.teamcode.utility.RotationUtil;
import org.firstinspires.ftc.teamcode.utility.point;
import org.firstinspires.ftc.teamcode.utility.pose;

import java.util.List;

public class OdometryThatusesIMUforHeading extends Odometry implements GivesPosition {

    IMU imu;
    double prevAngle = 0;

    protected void loop() {
        wheels.forEach(OdometryWheel::updateDelta);

        pose delta = getDeltaPose();

        double imuHeading = imu.getHeading();
        delta.r = RotationUtil.turnLeftOrRight(prevAngle, imuHeading, Math.PI/2);
        prevAngle = imuHeading;

        pose curved = curvedTrajectoryTranslation(new pose(delta.x, delta.y, delta.r));
        point itrans = point.rotate(curved.x, curved.y, -Math.PI/2);
        curved.x = -itrans.x;
        curved.y = -itrans.y;

        position.translateRelative(curved);
    }

    public OdometryThatusesIMUforHeading(IMU imu, pose initial, List<OdometryWheel> wheels) {
        super(initial, wheels);
        this.imu = imu;
    }
}
