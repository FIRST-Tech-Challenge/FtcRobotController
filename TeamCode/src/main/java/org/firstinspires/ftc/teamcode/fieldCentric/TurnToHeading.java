package org.firstinspires.ftc.teamcode.fieldCentric;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.MechanicalDriveBase;
import org.firstinspires.ftc.teamcode.IMU.IMUControl;

public class TurnToHeading
{
    Telemetry telemetry;
    MechanicalDriveBase mechanicalDriveBase;
    IMUControl imu;

//    PIDController pid;
    private double target;

    public TurnToHeading(Telemetry telemetry, MechanicalDriveBase mechanicalDriveBase, IMUControl imu)
    {
        this.telemetry = telemetry;
        this.mechanicalDriveBase = mechanicalDriveBase;
        this.imu= imu;
    }

    protected double turnToHeading(double x, double y, double deadzone_x, double deadzone_y)
    {
        if (Math.abs(x) <= deadzone_x && Math.abs(y) <= deadzone_y)
        {
            return(0);
        }
        double imu_heading = imu.getAngle();
        double current_heading = Math.signum(imu_heading) * Math.abs(imu_heading)%360;
        double target_heading = Math.toDegrees(Math.atan2(-x, -y));
        if (Math.signum(target_heading) == -1.0)
        {
            target_heading = target_heading + 360;
        }
        if (Math.signum(current_heading) == -1.0)
        {
            current_heading = current_heading + 360;
        }
        double delta_heading = current_heading + 360 - target_heading;
        if (Math.abs(current_heading - target_heading) <= Math.abs(current_heading + 360 - target_heading))
        {
            delta_heading = current_heading - target_heading;
            if (Math.abs(current_heading - target_heading) >= Math.abs(current_heading - 360 - target_heading))
            {
                delta_heading = current_heading - 360 - target_heading;
            }
        }
        delta_heading = delta_heading/90;
        telemetry.addData("cur heading", current_heading);
        telemetry.addData("target heading", target_heading);
        telemetry.addData("delta heading", delta_heading);
        telemetry.addData("x sign", Math.signum(x));
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
        return (delta_heading);
    }
/*
    {
        double imu_heading = imu.getAngle();
        imu_heading = Math.abs(imu_heading) % 360;
        //if (Math.abs(x) > deadzone && Math.abs(y) > deadzone)
        //{
            double target_heading = Math.toDegrees(Math.atan2(x, y));
            if (x != 0)
            {
                target_heading = target_heading * Math.signum(x);
            }
        //}

        telemetry.addData("cur heading", imu_heading);
        telemetry.addData("target heading", target_heading);
        telemetry.addData("x sign", Math.signum(x));
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
    }
*/
}
