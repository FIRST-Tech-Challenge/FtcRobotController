package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;

import org.firstinspires.ftc.teamcode.Util.Names;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = Names.DRIVE_FRONT_LEFT;
        FollowerConstants.leftRearMotorName = Names.DRIVE_BACK_LEFT;
        FollowerConstants.rightFrontMotorName = Names.DRIVE_FRONT_RIGHT;
        FollowerConstants.rightRearMotorName = Names.DRIVE_BACK_RIGHT;

        FollowerConstants.leftFrontMotorDirection = Names.DRIVE_FRONT_LEFT_DIRECTION;
        FollowerConstants.leftRearMotorDirection = Names.DRIVE_BACK_LEFT_DIRECTION;
        FollowerConstants.rightFrontMotorDirection = Names.DRIVE_FRONT_RIGHT_DIRECTION;
        FollowerConstants.rightRearMotorDirection = Names.DRIVE_BACK_RIGHT_DIRECTION;

        FollowerConstants.mass = 0;

        FollowerConstants.xMovement = 58.62645796861983;
        FollowerConstants.yMovement = 43.764789808914855;

        FollowerConstants.forwardZeroPowerAcceleration = -39.62117650174046;
        FollowerConstants.lateralZeroPowerAcceleration = -73.3436749528114;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0,0,0,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0,0,0,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.008,0,0.0002,0,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0,0,0,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
