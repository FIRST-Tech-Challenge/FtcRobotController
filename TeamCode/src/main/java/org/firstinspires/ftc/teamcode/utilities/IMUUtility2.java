package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMUUtility2 {

    public IMU imu;

    public Orientation             lastAngles = new Orientation();
    public double                  desiredGlobalAngle_d;
//    public BNO055IMU        imu;
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public OpMode _opMode;

    public void initialize(OpMode opMode, String configName) {
        _opMode = opMode;
        imu = _opMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters2 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters2);


//        this.resetAngle();
    }

    public Orientation getCurrentAngle(){
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return angles;
    }

    public double getCurrentAbsoluteAngle(){
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getCurrentAbsoluteAngle2(){
        Orientation angles = imu.getRobotOrientation(AxesReference.EXTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    public double getCurrentAbsoluteAngle3(){
        Orientation angles = imu.getRobotOrientation(AxesReference.EXTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return angles.thirdAngle;
    }


    public double getAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//        RobotLog.i(String.format("Prev Angle %f Current angle %f",lastAngles.firstAngle,angles.firstAngle));

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        desiredGlobalAngle_d += deltaAngle;

        lastAngles = angles;

        return desiredGlobalAngle_d;
    }
    public double getAngleX(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastAngles.thirdAngle;
        RobotLog.i(String.format("Prev Angle %f Current angle %f",lastAngles.thirdAngle,angles.thirdAngle));

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        desiredGlobalAngle_d += deltaAngle;

        lastAngles = angles;

        return desiredGlobalAngle_d;
    }
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    public double checkDirectionX()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngleX();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    public void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        imu.resetYaw();
//        RobotLog.i(String.format("RESETTING IMU ANGLE: %f",lastAngles.firstAngle));
        desiredGlobalAngle_d = 0;
    }
}
