package org.firstinspires.ftc.teamcode.utility;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class OdometryInfo {
    private Pose2d pos;
    private Rotation2d rot;

    public Pose2d getPos() {
        return pos;
    }

    public void setPos(Pose2d pos) {
        this.pos = pos;
    }

    public Rotation2d getRot() {
        return rot;
    }

    public void setRot(Rotation2d rot) {
        this.rot = rot;
    }
}
