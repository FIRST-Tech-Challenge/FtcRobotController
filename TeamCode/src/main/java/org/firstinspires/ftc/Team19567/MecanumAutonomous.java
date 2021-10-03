package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Mecanum Autonomous", group="Linear Opmode")

public class MecanumAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo servo1 = null;
    private BNO055IMU imu = null;
    private final static double TICKS_PER_REVOLUTION = 537.7; //TPR, RPI, and TPI are all for GoBilda 312 RPM 19:2 planetary Yellow Jacket motors
    private final static double REVOLUTIONS_PER_INCH = 1/11.873736;
    private final static double TICKS_PER_INCH = TICKS_PER_REVOLUTION*REVOLUTIONS_PER_INCH;
    private static final double minAccel = 0.3;
    private static final double minDecel = 0.1;
    private static final double ticksAccel = 600.0;
    private static final double ticksDecel = 1200.0;
    private static final double degsAccel = 20.0;
    private static final double degsDecel = 30.0;
    private double heading;             // the current heading of the robot
    private int heading_revs = 0;       // the complete revolutions of the robot
    private double heading_raw_last;    // the last raw heading from the IMU
    private static final double forwardBias = 0.0;
    private static final double turningBias = 0.0;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        leftDCFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.FORWARD);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.FORWARD);

        double leftFrontSpeed = 0.0;
        double rightFrontSpeed = 0.0;
        double leftBackSpeed = 0.0;
        double rightBackSpeed = 0.0;

        //Set DC motors to run with encoder

        resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //Wait for the driver to press Init
        runtime.reset(); //Reset the runtime

        mecanumStrafe(10, 0,0.7);
        mecanumRotate(45, 0.4);
        mecanumStrafe(5,45,0.6);
        mecanumRotate(-180,0.5);
        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }

    public void resetEncoders() {
        leftDCFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDCFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDCBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDCBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDCFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDCFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDCBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDCBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double forward_tics() {
        return rightDCFront.getCurrentPosition() + leftDCFront.getCurrentPosition() +
                rightDCBack.getCurrentPosition() + leftDCBack.getCurrentPosition();
    }

    private double sideways_tics() {
        return (rightDCBack.getCurrentPosition() + leftDCFront.getCurrentPosition()) -
                (rightDCFront.getCurrentPosition() + leftDCBack.getCurrentPosition());
    }

    protected double powerMultiplier (double current, double target,
                                       double mtr_accel_min, double mtr_decel_min,
                                       double accel, double decel) {
        if (current <= 0.0) {
            // Not yet at the expected start. This could happen if there was some robot
            // motion (was hit or coasting) that confused the sensor/logic. In this
            // case, move at the minimum power until the caller knows what's happening.
            return mtr_accel_min;
        } else if (current >= target) {
            // Past the expected target. This could happen if there was some robot motion
            // (was hit or coasting) that confused the sensor/logic. In this case stop.
            return 0.0;
        }
        double mtr_tmp = 1.0;
        if (current < accel) {
            // in the acceleration zone
            mtr_tmp = mtr_accel_min + (1.0 - mtr_accel_min) * (current / accel);
        }
        if (current > target - decel) {
            // in the deceleration zone
            double mtr_tmp_2 = mtr_decel_min +
                    (1.0 - mtr_decel_min) * ((target - current) / decel);
            if (mtr_tmp_2 < mtr_tmp) {
                // Could also be in the acceleration zone - in this case the deceleration
                // value is less than the acceleration or the 1.0 default.
                mtr_tmp = mtr_tmp_2;
            }
        }
        return mtr_tmp;
    }

    protected void lclSetPower(double power_rf, double power_rr, double power_lf, double power_lr) {
        rightDCFront.setPower(power_rf);
        rightDCBack.setPower(power_rr);
        leftDCFront.setPower(power_lf);
        leftDCBack.setPower(power_lr);

        // anytime power is being changed the heading may change. The IMU goes from
        // -180.0 to 180.0. The discontinuity at 180,-180 is a programming headache.
        // if you rotate through that is takes a bunch of special programming logic
        // to figure out where you are. Instead, we will monitor going through that
        // discontinuity and increment a rotation counter so our heading will start
        // at 0 when the IMU is initialized, and be a continuous function from
        // -infinity to +infinity.
        Orientation angles = imu.getAngularOrientation();
        double heading_raw = angles.firstAngle;
        if (heading_raw_last < -140.0 && heading_raw > 0.0) {
            heading_revs -= 1;
        } else if (heading_raw_last > 140.0 && heading_raw < 0.0) {
            heading_revs += 1;
        }
        // Our mental model says clockwise rotation (turning right) is a positive
        // rotation for the power, so we will sign correct heading to match.
        heading = -(heading_revs * 360.0 + heading_raw);
        heading_raw_last = heading_raw;
        telemetry.addData("Heading", heading);
        telemetry.update();

    }

    public void setSpeeds(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        double scale = 1.0;
        double max = Math.abs(forward) +
                Math.abs(sideways * (Math.abs(forwardBias) + Math.abs(turningBias) + 1.0)) +
                Math.abs(rotate);
        if (max > 1.0) {
            scale = 1.0 / max;
        }
        // Compute the power to each of the motors
        double power_rf = scale *
                (forward +
                        sideways * (forwardBias - turningBias - 1.0) -
                        rotate);
        double power_rr = scale *
                (forward +
                        sideways * (forwardBias - turningBias + 1.0) -
                        rotate);
        double power_lf = scale *
                (forward +
                        sideways * (forwardBias + turningBias + 1.0) +
                        rotate);
        double power_lr = scale *
                (forward +
                        sideways * (forwardBias + turningBias - 1.0) +
                        rotate);
        // set the powers to each of the motors
        lclSetPower(power_rf, power_rr, power_lf, power_lr);
    }

    public void mecanumStrafe(double inches, double degrees, double max_speed) {
        resetEncoders();
        double radians = Math.toRadians(degrees);
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);
        double forward_max_speed = Math.abs(cos * max_speed);
        double forward_inches = cos * inches;
        double forward_direction_mult = (forward_inches > 0.0) ? 1.0 : -1.0;
        double sideways_max_speed = Math.abs(sin * max_speed);
        double sideways_inches = sin * inches;
        double sideways_direction_mult = (sideways_inches > 0.0) ? 1.0 : -1.0;
        double target_tics = (forward_max_speed >= sideways_max_speed) ?
                (TICKS_PER_INCH * forward_inches * forward_direction_mult) :
                (TICKS_PER_INCH * sideways_inches * sideways_direction_mult);
        while (true) {
            double current_tics = (forward_max_speed >= sideways_max_speed) ?
                    (forward_tics() * forward_direction_mult) : (sideways_tics() * sideways_direction_mult);
            if (current_tics > target_tics) {
                break;
            }
            double speed_mult = powerMultiplier(current_tics, target_tics, minAccel, minDecel,
                    ticksAccel, ticksDecel);
            setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                    sideways_max_speed * speed_mult * sideways_direction_mult, 0.0);
        }
        setSpeeds(0.0, 0.0, 0.0);
    }
    public void mecanumRotate(double degrees, double max_rotate_speed) {
        double start_heading = heading;
        // Rotate as specified - there is normally some overshoot.
        turn(degrees);
        // Test the heading and correct for error (overshoot)
        turn(degrees - (heading - start_heading));
    }

    private void turn(double degrees) {
        resetEncoders();
        double direction_mult = (degrees > 0.0) ? 1.0 : -1.0;
        double start_heading = heading;
        double target = degrees * direction_mult;
        while (true) {
            double current = direction_mult * (heading - start_heading);
            if (current >= target) {
                break;
            }
            setSpeeds(0.0, 0.0,
                    direction_mult * powerMultiplier(current, target,
                            minAccel, minDecel,
                            degsAccel, degsDecel));
        }
        setSpeeds(0.0, 0.0, 0.0);
    }
}