package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * ODOMETRY - where the robot is on the field (goBILDA Pinpoint).
 *
 * X is forward and Y is left, both in inches. Heading is in degrees,
 * and turning counterclockwise (left) makes it go up.
 * The pose starts at 0, 0, 0 when the robot is initialized.
 *
 * Put here:
 *  - Pinpoint setup in init()
 *  - update() that refreshes the position every loop
 *  - getters other code can read
 * Do NOT put here: driving. Other code reads the pose and decides what to do.
 */
public class Odometry implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
        pinpoint.setOffsets(RobotConstants.PINPOINT_X_OFFSET_MM,
                RobotConstants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(RobotConstants.POD_TYPE);
        pinpoint.setEncoderDirections(RobotConstants.X_POD_DIRECTION, RobotConstants.Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU(); // takes a moment, so keep the robot still during INIT
    }

    @Override
    public void update() {
        pinpoint.update();
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f",
                getXInches(), getYInches(), getHeadingDegrees());
    }

    @Override
    public void stop() {
    }

    public double getXInches() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeadingDegrees() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
}
