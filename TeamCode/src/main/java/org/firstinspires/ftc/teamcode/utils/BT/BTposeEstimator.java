package org.firstinspires.ftc.teamcode.utils.BT;
import org.firstinspires.ftc.teamcode.utils.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.utils.geometry.*;


import java.util.function.DoubleSupplier;

public class BTposeEstimator extends HolonomicOdometry {

    public BTposeEstimator(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder, DoubleSupplier horizontalEncoder, DoubleSupplier gyroAngle, double trackWidth, double centerWheelOffset) {
        super(leftEncoder, rightEncoder, horizontalEncoder, gyroAngle, trackWidth, centerWheelOffset);
    }

    public void setPoseToCameraPose(BTPose2d cameraPose){
        updatePose();
        robotPose = cameraPose;

    }
}
