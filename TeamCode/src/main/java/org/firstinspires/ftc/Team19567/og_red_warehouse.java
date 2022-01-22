package org.firstinspires.ftc.Team19567;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="OG Red Warehouse", group="Linear Opmode")

public class og_red_warehouse extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotor linearSlideDC = null;
    private DcMotor carouselDC = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo releaseServo = null;
    private BNO055IMU imu = null;
    private final static double TICKS_PER_REVOLUTION = 537.7; //TPR, RPI, and TPI are all for GoBilda 312 RPM 19:2 planetary Yellow Jacket motors
    private final static double REVOLUTIONS_PER_INCH = 0.5;
    private final static double TICKS_PER_INCH = TICKS_PER_REVOLUTION * REVOLUTIONS_PER_INCH;
    private static final double minAccel = 0.3;
    private static final double minDecel = 0.1;
    private static final double ticksAccel = 600.0;
    private static final double ticksDecel = 1200.0;
    private static final double degsAccel = 20.0;
    private static final double degsDecel = 15.0;
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
        linearSlideDC = hardwareMap.get(DcMotor.class, "linearSlideDC");
        carouselDC = hardwareMap.get(DcMotor.class, "carouselDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Set direction to be forward in case the robot's motors are oriented otherwise; can change FORWARD to REVERSE if necessary

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.REVERSE);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.REVERSE);
        carouselDC.setDirection(DcMotor.Direction.FORWARD);

        //Set DC motors to run with encoder
        resetEncoders();

        linearSlideDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //Wait for the driver to press Init
        runtime.reset(); //Reset the runtime

        if(!opModeIsActive()) return;

        mecanumStrafe(11,0,0.7);
        sleep(1000);
        changeDirection();
        mecanumRotate(70,0.5);
        sleep(2000);
        changeDirection();
        mecanumStrafe(30,0,0.7);
        sleep(3000);

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

    private void changeDirection() {
        if(leftDCBack.getDirection() == DcMotor.Direction.FORWARD) {
            leftDCFront.setDirection(DcMotor.Direction.REVERSE);
            rightDCFront.setDirection(DcMotor.Direction.FORWARD);
            leftDCBack.setDirection(DcMotor.Direction.REVERSE);
            rightDCBack.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            leftDCFront.setDirection(DcMotor.Direction.FORWARD);
            rightDCFront.setDirection(DcMotor.Direction.REVERSE);
            leftDCBack.setDirection(DcMotor.Direction.FORWARD);
            rightDCBack.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private double forward_tics() {
        return rightDCFront.getCurrentPosition() + leftDCFront.getCurrentPosition() +
                rightDCBack.getCurrentPosition() + leftDCBack.getCurrentPosition();
    }

    private double sideways_tics() {
        return (rightDCBack.getCurrentPosition() + leftDCFront.getCurrentPosition()) -
                (rightDCFront.getCurrentPosition() + leftDCBack.getCurrentPosition());
    }

    protected double powerMultiplier(double current, double target,
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
        double forward_max_speed = cos * max_speed;
        double forward_inches = cos * inches;
        double forward_direction_mult = (forward_inches > 0.0) ? 1.0 : -1.0;
        double sideways_max_speed = sin * max_speed;
        double sideways_inches = sin * inches;
        double sideways_direction_mult = (sideways_inches > 0.0) ? 1.0 : -1.0;
        double target_tics = (Math.abs(forward_max_speed) >= Math.abs(sideways_max_speed)) ?
                (Math.abs(TICKS_PER_INCH * forward_inches) * forward_direction_mult) :
                (Math.abs(TICKS_PER_INCH * sideways_inches) * sideways_direction_mult);
        telemetry.addData("Inches", "forward_inches(%.2f), sideways_inches(%.2f)",forward_inches,sideways_inches);
        telemetry.addData("Target","Target(%.2f)",target_tics);
        telemetry.addData("MaxSpeeds","forward_max_speed(%.2f), sideways_max_speed(%.2f)",forward_max_speed,sideways_max_speed);
        telemetry.addData("Motors","Forward(%.2f),Sideways(%.2f)",forward_direction_mult,sideways_direction_mult);
        telemetry.update();
        sleep(1000);
        while (opModeIsActive()) {
            double forward_tics = forward_tics();
            double sideways_tics = sideways_tics();
            double current_tics = (Math.abs(forward_max_speed) >= Math.abs(sideways_max_speed)) ?
                    (forward_tics * forward_direction_mult) : (sideways_tics * sideways_direction_mult);
            telemetry.addData("Current","current_tics(%.2f), forward_tics(%.2f),sideways_tics(%.2f)",current_tics,forward_tics,sideways_tics);
            telemetry.update();
            if ((target_tics > 0 && current_tics > target_tics) || (target_tics < 0 && current_tics < target_tics) || target_tics == 0) {
                break;
            }
            double speed_mult = powerMultiplier(current_tics, target_tics, minAccel, minDecel,
                    ticksAccel, ticksDecel);
            telemetry.addData("speed_mult","speed_mult(%.2f)",speed_mult);
            setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                    sideways_max_speed * speed_mult * sideways_direction_mult, 0.0);
        }
        setSpeeds(0.0, 0.0, 0.0);
    }

    public void mecanumRotate(double degrees, double max_rotate_speed) {
        double start_heading = heading;
        // Rotate as specified - there is normally some overshoot.
        turn(degrees);
    }

    private void turn(double degrees) {
        resetEncoders();
        double direction_mult = (degrees > 0.0) ? 1.0 : -1.0;
        double start_heading = heading;
        double target = degrees * direction_mult;
        while (opModeIsActive()) {
            double current = direction_mult * (heading - start_heading);
            if (Math.abs(current) >= target) {
                break;
            }
            setSpeeds(0.0, 0.0,
                    direction_mult * powerMultiplier(current, target,
                            minAccel, minDecel,
                            degsAccel, degsDecel));
        }
        setSpeeds(0.0, 0.0, 0.0);
    }

    private void carouselMove(double speed) {
       carouselDC.setPower(speed);
    }

    private void linearSlideMove(int position, double speed) {
        linearSlideDC.setPower(speed);
        linearSlideDC.setTargetPosition(position);
        linearSlideDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void releaseServoMove(double position) {
        releaseServo.setPosition(position);
    }
}