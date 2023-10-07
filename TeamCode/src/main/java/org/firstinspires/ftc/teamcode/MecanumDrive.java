//  _____                _           _       _  _    __  _____ _  _
// |  ___| __ ___   __ _| |__   ___ | |_ ___| || |  / /_|___ /| || |
// | |_ | '__/ _ \ / _` | '_ \ / _ \| __/ __| || |_| '_ \ |_ \| || |_
// |  _|| | | (_) | (_| | |_) | (_) | |_\__ \__   _| (_) |__) |__   _|
// |_|  |_|  \___/ \__, |_.__/ \___/ \__|___/  |_|  \___/____/   |_|
//                 |___/
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.DriveTrain;
import org.firstinspires.ftc.teamcode.drivebase.MotorPowers;

/**
 * Created by michael on 11/12/18.
 */

public class MecanumDrive
{
    public static void cartesian(CenterStageDriveBase driveTrain, double main, double strafe, double turn)
    {
        driveTrain.setMotorPowers(calcCartesian(main, strafe, turn));
    }

    public static MotorPowers calcCartesian(double main, double strafe, double turn)
    {
        MotorPowers motorPowers = new MotorPowers();

        //Run the holonomic formulas for each wheel
        motorPowers.frontLeft  = main + strafe + turn;
        motorPowers.frontRight = main - strafe - turn;
        motorPowers.rearLeft   = main - strafe + turn;
        motorPowers.rearRight  = main + strafe - turn;

        //normalize(motorPowers); NOTE: moved to DT layer

        return motorPowers;
    }

    public static void polar(DriveTrain driveTrain, double mag, double dir, double rot)
    {
        driveTrain.setMotorPowers(calcPolar(mag, dir, rot));
    }

    public static MotorPowers calcPolar(double mag, double dir, double rot)
    {
        /*
         * Formula ported from WPI-lib
         */

        MotorPowers powers = new MotorPowers();

        //Account for the fact that the sine of a 90 degree angle is
        //1 but the sine of a 45 degree angle is sqrt(2) / 2
        //Thus if we didn't do anything, commanding, for example, full
        //power at 90 deg would result in only sqrt(2) / 2 power being
        //applied, but commanding full power at 45 deg would indeed
        //apply full power.
        mag = limit(mag, 1.0) * Math.sqrt(2);

        //Convert input angle to radians (Java trig functions use radians)
        double dirRad = Math.toRadians(dir + 45); //Offset by 45 due angle of rollers

        //Compute the power for each wheel
        powers.frontLeft  =  mag * Math.sin(dirRad) + rot;
        powers.frontRight = -mag * Math.cos(dirRad) - rot; //Negate mag because motor should already be set to reverse
        powers.rearLeft   = -mag * Math.cos(dirRad) + rot; // ^ ^ ^
        powers.rearRight  =  mag * Math.sin(dirRad) - rot;

        //Because we multiplied by sqrt(2) above, there is the possibility
        //of computing a power greater than +-1. Thus, we need to normalize
        //everything to bring it back in range. If we just clipped, then the
        //robot would not be following the vector correctly because the ratios
        //of the power would be wrong.
        //Although, adding the rotation power could also cause a power greater
        //than +- to be computed, so even if we hadn't multiplied by sqrt(2)
        //we should still normalize
        //normalize(powers); NOTE: MOVED TO DT LAYER

        return powers;
    }

    private static double limit(double num, double lim)
    {
        if (num > lim)
        {
            return lim;
        }

        else if (num < -lim)
        {
            return -lim;
        }

        return num;
    }
}