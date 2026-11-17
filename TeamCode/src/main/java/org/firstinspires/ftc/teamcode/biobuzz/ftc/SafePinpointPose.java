package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/** Validate raw odometry before constructing/normalizing an SDK pose. */
public final class SafePinpointPose {
    private SafePinpointPose() { }
    public static Pose2D read(GoBildaPinpointDriver device) {
        checkInterrupted();
        if(device.getDeviceStatus()!=GoBildaPinpointDriver.DeviceStatus.READY)return null;
        double x=device.getPosX(DistanceUnit.INCH),y=device.getPosY(DistanceUnit.INCH);
        double h=device.getHeading(UnnormalizedAngleUnit.RADIANS);
        checkInterrupted();
        org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose sample=org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose.validatedOdometry(x,y,h);
        return sample==null?null:new Pose2D(DistanceUnit.INCH,sample.x,sample.y,AngleUnit.RADIANS,sample.heading);
    }
    public static void checkInterrupted() {
        if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("Hardware operation cancelled by STOP");
    }
}
