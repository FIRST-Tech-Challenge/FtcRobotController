package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class moveUtils {

    // Things that need to be imported
    private static DcMotor LF = null;
    private static DcMotor RF = null;
    private static DcMotor LB = null;
    private static DcMotor RB = null;
    private static CRServo spinner = null;
    private static BNO055IMU imu;
    private static float desiredHeading;
    private static CRServo spinspinducky = null;
    private static Servo dumper = null;
    private static DcMotor armboom = null;

    // Things specific to this class
    private static final float TURN_SPEED_HIGH = 1f;
    private static final float TURN_SPEED_LOW = 0.15f;
    private static final float TURN_HIGH_ANGLE = 45.0f;
    private static final float TURN_LOW_ANGLE = 5.0f;

    static final float EncoderTicks = 537.6f;
    static final float WHEEL_DIAMETER_INCHES = 4.0f;
    static final float REVS_PER_INCH_MOD = 50f/49f;
    static final float COUNTS_PER_INCH = (EncoderTicks * REVS_PER_INCH_MOD) / (3.1416f * WHEEL_DIAMETER_INCHES);
    static final float SCALE_ADJUST_FWD = 3.0f;

    static final float STRAFE_MOD = 18f;
    static final float MAX_STRAFE_SPEED = 1.0f;

    public static void initialize(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, BNO055IMU imu, float currHeading) {
        moveUtils.LF = LF;
        moveUtils.RF = RF;
        moveUtils.LB = LB;
        moveUtils.RB = RB;
        moveUtils.spinner = spinner;
        moveUtils.imu = imu;
        moveUtils.desiredHeading = currHeading;
    }

    public static void turnCW(float turnDegrees) {
        desiredHeading -= turnDegrees;
        if (desiredHeading < -180) {
            desiredHeading += 360;
        }
        turnToHeading();
    }

    public static void turnCCW(float turnDegrees) {
        desiredHeading += turnDegrees;
        if (desiredHeading > 180) {
            desiredHeading -= 360;
        }
        turnToHeading();
    }

    public static void turnToHeading() {
        boolean isCW = deltaHeading() > 0;

        if (isCW) {
            // 1st stage - high power rough heading.
            if (deltaHeading() > TURN_HIGH_ANGLE) {
                setAllMotorsTurnPower(TURN_SPEED_HIGH);
                while (deltaHeading() > TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() > TURN_LOW_ANGLE) {
                setAllMotorsTurnPower(TURN_SPEED_LOW);
                while (deltaHeading() > TURN_LOW_ANGLE) {
                }
            }
        } else { // Going CCW
            // 1st stage - high power rough heading.
            if (deltaHeading() < -TURN_HIGH_ANGLE) {
                setAllMotorsTurnPower(-TURN_SPEED_HIGH);
                while (deltaHeading() < -TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() < -TURN_LOW_ANGLE) {
                setAllMotorsTurnPower(-TURN_SPEED_LOW);
                while (deltaHeading() < -TURN_LOW_ANGLE) {
                }
            }
        }
        resetEncoders();
    }

    private static void setAllMotorsTurnPower(float turnPower) {
        LF.setPower(turnPower);
        LB.setPower(turnPower);
        RF.setPower(-turnPower);
        RB.setPower(-turnPower);
    }

    private static void setAllMotorsStraightPower(float turnPower) {
        LF.setPower(turnPower);
        LB.setPower(turnPower);
        RF.setPower(turnPower);
        RB.setPower(turnPower);
    }

    private static float deltaHeading() {
        float dH = getHeading() - desiredHeading;
        if (dH < -180) { dH += 360; }
        if (dH > 180) { dH -= 360; }
        return dH;
    }

    public static float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                DEGREES);
        return angles.firstAngle;
    }

    public static void resetEncoders() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Just a little time to make sure encoders have reset
        //sleep(200);

        // Not technically encoder but...
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Only using the Back motor Encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void goStraight(float totalDistIn, float maxPower, float minPower, int accel) {
        int distance;
        int rampUpDist;
        int rampDownDist;
        float currentPower;
        int currentDistLB = 0;
        int currentDistRB = 0;
        int encoderDiff;  // difference in LB and RB wheel encoder count
        double powerL;  // modified Left-side motor power to equalize motors
        double powerR;  // modified Right-side motor power to equalize motors
        boolean isForward = false;

        // Use this to determine to go backward or forward
        // + totalDistIn means go forward 'totalDistIn' inches
        // - totalDistIn means go backward 'totalDistIn' inches

        if (totalDistIn > 0) {
            isForward = true;
        }


        // Convert inches to encoder ticks
        distance = (int) (Math.abs(totalDistIn) * COUNTS_PER_INCH);  // distance is encoder ticks, not inches
        rampUpDist = (int) ((maxPower - minPower) * 100 * accel);  // calculates number of encoder ticks (distance) to get to full speed
        rampDownDist = distance - rampUpDist;  // calculates when (in encoder ticks) to start slowing down

        // Need our ramp-up distance to be less than half or else would not have time to decelerate
        if (rampUpDist > distance / 2) {
            rampUpDist = distance / 2;
            rampDownDist = distance / 2;
        }

        // Prepare motor encoders, turns off since not running to set position
        // Calculating power instead
        resetEncoders();

        // Setting power to motors
        currentPower = minPower;
        if (isForward) {
            setAllMotorsStraightPower(currentPower);
        } else {
            setAllMotorsStraightPower(-currentPower);
        }
        // MKing - go forward or backward AND use encoder comparison code for error correction!
        while (currentDistLB < distance) {  // While distance not met
            if (currentDistLB < rampUpDist) {  // Accelerating
                currentPower = minPower + (currentDistLB / accel) / 100.0f;
                maxPower = currentPower;
            } else if (currentDistRB >= rampDownDist) {  // Decelerating
                currentPower = maxPower - ((currentDistLB - rampDownDist) / accel) / 100.0f;
            }

            currentDistLB = Math.abs(LB.getCurrentPosition());
            currentDistRB = Math.abs(RB.getCurrentPosition());

            // MKing - code for encoder comparison error correcting to run straight!
            if (currentDistLB < currentDistRB) {  // Left side is lagging right side
                encoderDiff = currentDistRB - currentDistLB;
                powerL = currentPower;
                powerR = currentPower * ((100.0 - (encoderDiff * SCALE_ADJUST_FWD)) / 100.0);
            } else {  // Right side is lagging left side
                encoderDiff = currentDistLB - currentDistRB;
                powerR = currentPower;
                powerL = currentPower * ((100.0 - (encoderDiff * SCALE_ADJUST_FWD)) / 100.0);
            }

            if (isForward) {
                LF.setPower(powerL);
                RF.setPower(powerR);
                LB.setPower(powerL);
                RB.setPower(powerR);
            } else {
                LF.setPower(-powerL);
                RF.setPower(-powerR);
                LB.setPower(-powerL);
                RB.setPower(-powerR);
            }
        }
        resetEncoders();
    }

    public static void strafeBuddy(float distanceMoveInches) {

        distanceMoveInches*=STRAFE_MOD;

        if (distanceMoveInches > 0) {
            while ((LB.getCurrentPosition() < (distanceMoveInches)) && RB.getCurrentPosition() < (distanceMoveInches)) {
                LF.setPower(MAX_STRAFE_SPEED);
                RF.setPower(-MAX_STRAFE_SPEED);
                RB.setPower(MAX_STRAFE_SPEED);
                LB.setPower(-MAX_STRAFE_SPEED);
            }
        } else {
            distanceMoveInches = 0-distanceMoveInches;
            while (LB.getCurrentPosition() < (distanceMoveInches) && RB.getCurrentPosition() < (distanceMoveInches)) {
                LF.setPower(-MAX_STRAFE_SPEED);
                RF.setPower(MAX_STRAFE_SPEED);
                RB.setPower(-MAX_STRAFE_SPEED);
                LB.setPower(MAX_STRAFE_SPEED);
            }
        }

        // Once the strafe is complete, reset the state of the motors.

        resetEncoders();

        // And make sure we haven't drifted on heading.
        turnToHeading();
    }


}
