package org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.Vector;

import org.firstinspires.ftc.teamcode.AutoCode.Control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.AutoCode.Control.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivebase.MotorPowers;
import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;

public class Waypoint implements MovementPerformer
{
    double targetX;
    double targetY;
    double maxPower;
    double targetHeading;
    AcceleratedGain acclHeadingGain;
    double headingGain;
    double maxTurnPower = 0.25;
    double thresh;
    double angleThresh;
    double speed;

    TrackingWheelIntegrator trackingWheelIntegrator;
    CenterStageDriveBase skyStoneDriveBase;

    TranslationThreshMethod translationThreshMethod;

    public enum TranslationThreshMethod
    {
        MAG,
        X_ONLY,
        Y_ONLY,
        X_ERR_FLIP,
        Y_ERR_FLIP
    }

    public Waypoint()
    {
        trackingWheelIntegrator = Globals.trackingWheelIntegrator;
        skyStoneDriveBase = Globals.driveBase;
    }

    @Override
    public void run()
    {
        moveThroughWaypoint();
    }

    void moveThroughWaypoint()
    {
        boolean angleGood = false;
        boolean translationGood = false;

        boolean xStartNegative = (targetX - trackingWheelIntegrator.getX()) < 0;
        boolean yStartNegative = (targetY - trackingWheelIntegrator.getY()) < 0;

        while ((!translationGood || !angleGood) && opModeIsActive())
        {
            updateTracking();

            double turnError = trackingWheelIntegrator.getHeading() - targetHeading;
            double xErr = targetX - trackingWheelIntegrator.getX();
            double yErr = targetY - trackingWheelIntegrator.getY();

            if(angleThresh == 0)
            {
                angleGood = true;
            }
            else
            {
                angleGood = Math.abs(turnError) < angleThresh;
            }

            if(translationThreshMethod == TranslationThreshMethod.X_ONLY)
            {
                translationGood = Math.abs(xErr) < thresh;
            }
            else if(translationThreshMethod == TranslationThreshMethod.Y_ONLY)
            {
                translationGood = Math.abs(yErr) < thresh;
            }
            else if(translationThreshMethod == TranslationThreshMethod.MAG)
            {
                double error = Vector.calcMag(xErr, yErr);
                translationGood = Math.abs(error) < thresh;
            }
            else if(translationThreshMethod == TranslationThreshMethod.X_ERR_FLIP)
            {
                translationGood = (xErr < 0) != xStartNegative;
            }
            else if(translationThreshMethod == TranslationThreshMethod.Y_ERR_FLIP)
            {
                translationGood = (yErr < 0) != yStartNegative;
            }

            Vector driveVector = new Vector();
            driveVector.addCartesian(xErr, yErr);
            driveVector.rotateDegrees(-trackingWheelIntegrator.getHeading());

            driveVector.setCartesian(driveVector.getX()*1.5, driveVector.getY());

            double turnCorrection = Range.clip(turnError*getHeadingGain(), -maxTurnPower, maxTurnPower);

            MotorPowers pows = MecanumDrive.calcPolar(speed, driveVector.getDir(), turnCorrection);

            skyStoneDriveBase.setMotorPowers(pows);
        }
    }

    double getHeadingGain()
    {
        if(acclHeadingGain == null)
        {
            return headingGain;
        }
        else
        {
            return acclHeadingGain.getControlledGain();
        }
    }

    boolean opModeIsActive()
    {
        return Globals.opMode.opModeIsActive();
    }

    void updateTracking()
    {
        Globals.updateTracking();
    }

    public static class Builder
    {
        double targetX;
        double targetY;
        double maxPower;
        double targetHeading;
        AcceleratedGain acclHeadingGain;
        double headingGain = .012;
        double maxTurnPower = 0.25;
        double thresh;
        double angleThresh;
        double speed;
        TranslationThreshMethod translationThreshMethod = TranslationThreshMethod.MAG;
        boolean movementThreshSet = false;
        boolean threshMethodSet = false;

        public Builder setTargetPosition(double x, double y)
        {
            this.targetX = x;
            this.targetY = y;

            return this;
        }

        public Builder setTransThreshMethod(TranslationThreshMethod method)
        {
            if(threshMethodSet)
            {
                throw new IllegalStateException("Cannot set Thresh method twice!");
            }

            threshMethodSet = true;

            if(movementThreshSet && method != TranslationThreshMethod.MAG)
            {
                throw new IllegalStateException("Cannot set non-mag method after thresh has already been set!");
            }

            this.translationThreshMethod = method;
            return this;
        }

        public Builder setSpeed(double speed)
        {
            this.speed = speed;
            return this;
        }

        public Builder setMovementThresh(double thresh)
        {
//            if(translationThreshMethod != TranslationThreshMethod.MAG)
//            {
//                throw new IllegalStateException("Cannot set movement thresh for non-mag method!");
//            }

            this.thresh = thresh;
            movementThreshSet = true;
            return this;
        }

        public Builder setTargetHeading(double heading)
        {
            this.targetHeading = heading;
            return this;
        }

        public Builder setHeadingThreshold(double headingThreshold)
        {
            this.angleThresh = headingThreshold;
            return this;
        }

        public Waypoint build()
        {
            Waypoint waypoint = new Waypoint();
            waypoint.targetX          = targetX;
            waypoint.targetY          = targetY;
            waypoint.targetHeading    = targetHeading;
            waypoint.maxPower         = maxPower;
            waypoint.maxTurnPower     = maxTurnPower;
            waypoint.angleThresh      = angleThresh;
            waypoint.headingGain      = headingGain;
            waypoint.speed            = speed;
            waypoint.thresh           = thresh;
            waypoint.translationThreshMethod = translationThreshMethod;
            return waypoint;
        }
    }
}
