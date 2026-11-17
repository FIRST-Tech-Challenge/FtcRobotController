
package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Pinpoint {
    public RobotSettings robotSettings;
    GoBildaPinpointDriver odo;
    Pose2D pos;


    public double x;
    public double y;
    public double heading;
    public static double Xoffset = 1.71507874; // -5.21653543 inches
    public static double YOffset = 5.93625984; // 0.55413386 inches


/*    Pose convertedPose = new Pose(
            y + 72,
            72 - x,

            heading - Math.PI / 2
    );*/


    public Pinpoint(MechanumHardwareMap mechanumHardwareMap) {
        odo = mechanumHardwareMap.getOdo();
        odo.setOffsets(Xoffset, YOffset, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robotSettings = RobotSettings.load();
        odo.resetPosAndIMU();
    }

    public void update() {
        odo.update();
        pos = odo.getPosition();
        x = pos.getX(DistanceUnit.INCH);
        y = pos.getY(DistanceUnit.INCH);
        heading = pos.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading() {
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Pose2D getPos() {
        return pos;
    }

    public void resetIMU() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 141.5, 0, AngleUnit.DEGREES, 0));
    }

    // NEW COORDINATE SYSTEM

    // public double x;
    // public double y;
    // public double heading;
    // Pose convertedPose;

    // public Pinpoint(HWMap hwMap) {
    //     odo = hwMap.getOdo();
    //     odo.setOffsets(Xoffset, YOffset, DistanceUnit.MM);
    //     odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    //     odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //     robotSettings = RobotSettings.load();
    //
    //     // Initialize position values after odo is set up
    //     pos = odo.getPosition();
    //     x = pos.getX(DistanceUnit.INCH);
    //     y = pos.getY(DistanceUnit.INCH);
    //     heading = pos.getHeading(AngleUnit.RADIANS);
    //     convertedPose = new Pose(
    //             y + 72,
    //             72 - x,
    //             heading - Math.PI / 2
    //     );
    // }

    // public void update() {
    //     odo.update();
    //     pos = odo.getPosition();
    //     x = pos.getX(DistanceUnit.INCH);
    //     y = pos.getY(DistanceUnit.INCH);
    //     heading = pos.getHeading(AngleUnit.RADIANS);
    //     convertedPose = new Pose(
    //             y + 72,
    //             72 - x,
    //             heading - Math.PI / 2
    //     );
    // }
    //

}
