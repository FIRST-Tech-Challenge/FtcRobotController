package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConeTracker {

    final private double MAX_RANGE = 300;
    final private double MIN_RANGE = 50;
    final private double DEAD_BAND = 15;

    public  boolean coneDetected  = false;
    public  double  coneRange     = MAX_RANGE;
    public  double  coneDirection = 0;

    private LinearOpMode        myOpMode = null;
    private Rev2mDistanceSensor left    = null;
    private Rev2mDistanceSensor center  = null;
    private Rev2mDistanceSensor right   = null;

    private double  leftRange = 0;
    private double  centerRange = 0;
    private double  rightRange = 0;

    private boolean leftSeen   = false;
    private boolean centerSeen = false;
    private boolean rightSeen  = false;
    private int     numberSeen = 0;

    public ConeTracker(LinearOpMode opmode) {
        myOpMode = opmode;
        left   = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "left_range");
        center = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "center_range");
        right  = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "right_range");
    }

    /***
     * Read the range sensors and update all the related parameters.
     * Must be called once per control cycle
     */
    public void update() {
        leftRange   = left.getDistance(DistanceUnit.MM);
        centerRange = center.getDistance(DistanceUnit.MM);
        rightRange  = right.getDistance(DistanceUnit.MM);
        numberSeen = 0;

        // Determine all the individual cone ranges and also the closest range
        coneRange = MAX_RANGE;
        if (leftRange < MAX_RANGE) {
            leftSeen = true;
            numberSeen++;
            coneRange = Math.min(leftRange, coneRange);
        } else {
            leftSeen  = false;
            leftRange = 0;
        }

        if (centerRange < MAX_RANGE) {
            centerSeen = true;
            numberSeen++;
            coneRange = Math.min(centerRange, coneRange);
        } else {
            centerSeen  = false;
            centerRange = 0;
        }

        if (rightRange < MAX_RANGE) {
            rightSeen = true;
            numberSeen++;
            coneRange = Math.min(rightRange, coneRange);
        } else {
            rightSeen  = false;
            rightRange = 0;
        }

        coneDetected = (numberSeen > 0);

        // Determine the direction to the cone.
        // +1 is max CCW direction, 0 is straight ahead, -1 is max CW direction.

        coneDirection = 0;
        if (numberSeen == 1){
            if (leftSeen) {
                coneDirection = 1;
            } else if (rightSeen) {
                coneDirection = -1;
            }
        } else if (numberSeen == 2) {
            if (leftSeen && centerSeen) {
                coneDirection = 0.5;
            } else if (rightSeen && centerSeen) {
                coneDirection = -0.5;
            }
        } else {
            double rangeDifference = (leftRange - rightRange);
            if (rangeDifference > DEAD_BAND) {
                coneDirection = 0.2;
            } else if (rangeDifference < -DEAD_BAND) {
                coneDirection = -0.2;
            }
        }

    }

    public void showRanges() {
        myOpMode.telemetry.addData("cone found", coneDetected);
        myOpMode.telemetry.addData("left", leftRange);
        myOpMode.telemetry.addData("center", centerRange);
        myOpMode.telemetry.addData("right", rightRange);
        myOpMode.telemetry.addData("cone range", coneRange);
        myOpMode.telemetry.addData("cone direction", coneDirection);

    }
}
