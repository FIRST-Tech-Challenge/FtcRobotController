package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class UpdatePowerTypes{
    private static final double minPower = 0.15;
    private static final double maxPower = 0.8;

    public static void basicUpdatePower(DcMotor motor){basicUpdatePower(motor, 100, 10);}
    public static void basicUpdatePower(DcMotor motor, int startDistance, int endDistance){basicUpdatePower(motor, minPower, maxPower,startDistance, endDistance);}
    public static void basicUpdatePower(DcMotor motor, double minPower, double maxPower, int startDistance, int endDistance){
        // Get the difference
        int targetDifference = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        // Check if out of range
        if (targetDifference > startDistance) {
            motor.setPower(maxPower);
        }
        // Check if minimum distance
        if (targetDifference < endDistance) {
            motor.setPower(minPower);
        }
        // Calculate damping
        int distanceRange = startDistance - endDistance;
        double damping = (double) (targetDifference - endDistance) / distanceRange;
        double powerRange = maxPower - minPower;
        double power =  powerRange * damping + minPower;
        // Set power
        motor.setPower(power);
    }

    public static double startEndUpdatePower(DcMotorEx motor, int startPostionSave){
        return startEndUpdatePower(motor, startPostionSave, 100, 5, 100, 10);
    }
    public static double startEndUpdatePower(DcMotorEx motor, int startPostionSave, int decelDistance1, int offsetOfStart, int decelDistance2, int offsetOfEnd){
        offsetOfEnd--;
        offsetOfStart--;

        //if target is smaller
        //than going backwards
        if (motor.getTargetPosition()<startPostionSave){
            return nextPart(motor.getCurrentPosition()-motor.getTargetPosition(), startPostionSave-motor.getTargetPosition()-offsetOfStart,
                    startPostionSave-motor.getTargetPosition()-offsetOfStart-decelDistance1, decelDistance2+offsetOfEnd, offsetOfEnd);
        }
        //if target is bigger
        //than going forward
        if (motor.getTargetPosition()>startPostionSave){
            return nextPart(motor.getTargetPosition()-motor.getCurrentPosition(), motor.getTargetPosition()-(startPostionSave+offsetOfStart),
                    motor.getTargetPosition()-(startPostionSave+offsetOfStart+decelDistance1), decelDistance2+offsetOfEnd, offsetOfEnd);
        }
        return 0;
    }



    private static double nextPart(double cur, int startPostion, int endPosition, int startPos2, int endPos2) {

        // Check if out of range
        if (cur > startPostion) {
            return minPower;
        }
        if (cur < endPosition) {
            return endingDamp(cur, startPos2, endPos2);
        }
        // Calculate damping
        int decelDistance = startPostion - endPosition;
        double distanceOnDecel = cur - endPosition;
        double normalizedDistanceOnDecel = distanceOnDecel / decelDistance;
        double flipNormalizedDistanceOnDecel = 1 - normalizedDistanceOnDecel;
        double powerRange = maxPower - minPower;
        double power = powerRange * flipNormalizedDistanceOnDecel + minPower;

        return power;
    }
    private static double endingDamp(double cur, int startPos2, int endpos2) {

        // Check if out of range
        if (cur > startPos2) {
            return maxPower;
        }
        if (cur < endpos2) {
            return minPower;
        }

        // Calculate damping
        int accDistance = startPos2 - endpos2;
        double distanceOnAcc = cur - endpos2;
        double normalizedDistanceOnAcc = distanceOnAcc / accDistance;
        double powerRange = maxPower - minPower;
        double power = powerRange * normalizedDistanceOnAcc + minPower;

        return power;
    }
}

