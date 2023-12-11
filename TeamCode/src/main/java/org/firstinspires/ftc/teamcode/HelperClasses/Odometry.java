package org.firstinspires.ftc.teamcode.HelperClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Odometry extends Hardware
{
    public Odometry (HardwareMap hardwareMap)
    {
        super(hardwareMap);
    }

    //constructs localizer object using one parameter of a list of three wheel positions
    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer
            (
                    new ArrayList<>(Arrays.asList(
                            new Pose2d(5.958820361, 0, Math.PI / 2), //center wheel
                            new Pose2d(0, trackwidth/2, 0), //right wheel
                            new Pose2d(0, -trackwidth/2, 0))) //left wheel
            )
    {
        //overrides getWheelPositions method
        @Override
        public List<Double> getWheelPositions()
        {
            ArrayList<Double> wheelPositions = new ArrayList<>(3);
            wheelPositions.add(centerOdomTraveled);
            wheelPositions.add(leftOdomTraveled);
            wheelPositions.add(rightOdomTraveled);
            return wheelPositions;
        }
    };

    private int getDeltaLeftTicks()
    {
        try
        {
            int total=-leftOdom.getCurrentPosition();
            int oldPos = leftEncoderPos;
            leftEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    private int getDeltaRightTicks()
    {
        try
        {
            int total=rightOdom.getCurrentPosition();
            int oldPos = rightEncoderPos;
            rightEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    private int getDeltaCenterTicks()
    {
        try
        {
            int total=centerOdom.getCurrentPosition();
            int oldPos = centerEncoderPos;
            centerEncoderPos=total;
            return oldPos - total;
        }
        catch(Exception e)
        {
            return 0;
        }
    }

    public void updatePositionRoadRunner()
    {
        /*try
        {
            bulkData = expansionHub.getBulkInputData();
        }
        catch(Exception e)
        {
            return;
        }*/

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks()/ ODOM_TICKS_PER_IN );
        double deltaRightDist = -(getDeltaRightTicks()/ ODOM_TICKS_PER_IN );
        double deltaCenterDist = getDeltaCenterTicks()/ ODOM_TICKS_PER_IN;

        //adjusts for physical diffrences in pods
        if(deltaLeftDist < 0)
            deltaRightDist *= (1.00052425 * 0.99770514);
        else if (deltaLeftDist > 0)
            deltaRightDist *= (1.00076168 * 0.99859913);

        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        //add negatives here to reverse x or y directions
        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = odom.getPoseEstimate().component1();
        y = odom.getPoseEstimate().component2();
    }

    public void resetOdometry(double x, double y, double theta)
    {
        odom.setPoseEstimate(new Pose2d(-x, -y, theta));

        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;

        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Resets encoder values then sets them back to run without encoders because wheels and odometry are same pointer
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void softBrake()
    {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}
