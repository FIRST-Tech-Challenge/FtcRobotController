package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.util.Arrays;

import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Thread.sleep;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 *
 * A change, don't worry Hans I'll change this later
 * ANOTHER CHANGE (devastating) don't worry this might ve deleted later
 */
public class SwerveChassis extends Logger<SwerveChassis> implements Configurable {


    final private CoreSystem core;

    public enum DriveMode {
        STOP,      // not moving
        STRAIGHT,  // driving in a straight line utilizing orientation sensor to correct itself,
        //  all servos are set to the same position
        ROTATE,    // rotating in place, all servos are set on tangent lines to a circle drawn
        //  through the centers of 4 wheels.
        STEER, // motor power and servos in the direction of movement are controlled by
        //  the driver; opposite servos are in central position
        TANK //Backup tank drive. Controlled with left power and right power

    }

    public enum LineColor {
        BLUE,
        RED
    }

    public static final double cutoffPercent = .8;
    // distance between the centers of left and right wheels, inches
    private double track = 13.75;
    // distance between the centers of front and back wheels, inches
    private double wheelBase = 11.25;
    // wheel radius, inches
    private double wheelRadius = 2.0;
    // minimum power that should be applied to the wheel motors for robot to start moving
    private double minPower = 0.25;
    // maximum power that should be applied to the wheel motors
    private double maxPower = 1.0;
    // the ratio of the distance that should be drove with desired power
    private double bufferPercentage = 0.8;
    private double chassisAligmentPower = 0.25;

    private double maxRange = 127; // max range sensor detectable

    private WheelAssembly frontLeft;
    private WheelAssembly frontRight;
    private WheelAssembly backLeft;
    private WheelAssembly backRight;
    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    private WheelAssembly[] wheels = new WheelAssembly[4];
    public CombinedOrientationSensor orientationSensor;

    public DistanceSensor frontLeftRangeSensor;
    public DistanceSensor frontRightRangeSensor;
    public DistanceSensor backRangeSensor;
    public DistanceSensor leftRangeSensor;
    public DistanceSensor rightRangeSensor;
    public DistanceSensor leftHiRangeSensor;
    public DistanceSensor rightHiRangeSensor;

    public ColorSensor FRColor;
    public ColorSensor FLColor;
    DistanceSensor FRDistance;
    DistanceSensor FLDistance;


    public Telemetry tl;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation
    private double curHeading = 0;
    private boolean useScalePower = true;//
    private boolean swerveReverseDirection = false; // chassis front/back is reversed during Teleop
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor
    private boolean showColor = true;
    private boolean slowMode = false;
    final double TICKS_PER_CM = 537.6 / (4.32 * 2.54 * Math.PI); // 16.86; //number of encoder ticks per cm of driving
    public final double DEFAULT_FAST_SCALE = 1.0;
    public final double DEFAULT_SLOW_SCALE = 0.35;
    private double defaultScale = DEFAULT_FAST_SCALE;
    private double SCALE_INC_TICK = 0.05;
    private boolean isTankDrive = false;

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public boolean isTankDrive() {
        return isTankDrive;
    }

    public void enableImuTelemetry() {
        setImuTelemetry = true;
    }

    public void enableShowColors() {
        showColor = true;
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
        if (slowMode) {
            defaultScale = DEFAULT_SLOW_SCALE;
        } else {
            defaultScale = DEFAULT_FAST_SCALE;
        }
    }

    public double getDefaultScale() {
        return defaultScale;
    }

    public void setDefaultScale(double val) {
        if (val > 0.5) slowMode = false;
        else slowMode = true;
        defaultScale = val;
    }

    public void incDefaultScale() {
        defaultScale += SCALE_INC_TICK;
        if (defaultScale > 1)
            defaultScale = 1;
        if (defaultScale > 0.5) slowMode = false;
        else slowMode = true;
    }

    public void decDefaultScale() {
        defaultScale -= SCALE_INC_TICK;
        if (defaultScale < 0.2)
            defaultScale = 0.2;
        if (defaultScale > 0.5) slowMode = false;
        else slowMode = true;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getTrack() {
        return track;
    }

    public void setTrack(double track) {
        this.track = track;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    @Adjustable(min = 1.0, max = 5.0, step = 0.02)
    public double getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    @Adjustable(min = 0.0, max = 0.4, step = 0.01)
    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Adjustable(min = 0.2, max = 1.0, step = 0.01)
    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    @Override
    public String getUniqueName() {
        return "chassis";
    }

    /**
     * SwerveChassis constructor
     */
    public SwerveChassis(CoreSystem core) {
        this.core = core;
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration, boolean auto, boolean enableSensors) {
        // set up motors / sensors as wheel assemblies
        wheels[0] = frontLeft = new WheelAssembly(
                //configuration, "FrontLeft", DcMotor.Direction.FORWARD
                configuration, "FrontLeft", DcMotor.Direction.REVERSE  // V5.3 goBilda 5202 motor
        );
        wheels[1] = frontRight = new WheelAssembly(
                //configuration, "FrontRight", DcMotor.Direction.REVERSE
                configuration, "FrontRight", DcMotor.Direction.FORWARD // V5.3 goBilda 5202 motor
        );
        wheels[2] = backLeft = new WheelAssembly(
                //configuration, "BackLeft", DcMotor.Direction.FORWARD
                configuration, "BackLeft", DcMotor.Direction.REVERSE   // V5.3 goBilda 5202 motor
        );
        wheels[3] = backRight = new WheelAssembly(
                //configuration, "BackRight", DcMotor.Direction.REVERSE
                configuration, "BackRight", DcMotor.Direction.FORWARD  // V5.3 goBilda 5202 motor
        );

        if (auto || setImuTelemetry) {
            orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
            orientationSensor.configure(configuration.getHardwareMap(), false,"imu", "imu2");
        }

        if ((auto || setRangeSensorTelemetry) && enableSensors) {
            frontLeftRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "frontLeftRange");
            frontRightRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "frontRightRange");
            backRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "backRange");
            leftRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "leftRange");
            rightRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "rightRange");
            leftHiRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "leftHiRange");
            rightHiRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "rightHiRange");

            FRColor = configuration.getHardwareMap().get(ColorSensor.class, "FRColor");
            // FLColor = configuration.getHardwareMap().get(ColorSensor.class, "FLColor");
            FRDistance = configuration.getHardwareMap().get(DistanceSensor.class, "FRColor");
            // FLDistance = configuration.getHardwareMap().get(DistanceSensor.class, "FLColor");
        }
        // register chassis as configurable component
        configuration.register(this);
    }


    /***
     * returns distance of given range sensor
     * @param direction sensor direction
     * @return distance
     */
    public double getDistance(Direction direction) {
        double dist = 0;
        if (Thread.interrupted()) return dist;

        int count = 0;
        DistanceSensor rangeSensor;

        switch (direction) {
            case FRONT_LEFT:
                rangeSensor = frontLeftRangeSensor;
                break;
            case FRONT:
            case FRONT_RIGHT:
                rangeSensor = frontRightRangeSensor;
                break;
            case LEFT:
                rangeSensor = leftRangeSensor;
                break;
            case RIGHT:
                rangeSensor = rightRangeSensor;
                break;
            case BACK:
                rangeSensor = backRangeSensor;
                break;
            case RIGHT_HI:
                rangeSensor = rightHiRangeSensor;
                break;
            case LEFT_HI:
                rangeSensor = leftHiRangeSensor;
                break;
            default:
                rangeSensor = null;
        }

        if (rangeSensor == null)
            return 0;
        dist = rangeSensor.getDistance(DistanceUnit.CM);
        while (dist > maxRange && (++count) < 5) {
            dist = rangeSensor.getDistance(DistanceUnit.CM);
            // yield handler
            this.core.yield();
        }
        if (direction == Direction.FRONT) { // use both two front sensor to improve the accuracy
            dist = Math.min(dist, frontLeftRangeSensor.getDistance(DistanceUnit.CM));
        }
        if (dist > maxRange)
            dist = maxRange;
        return dist;
    }

    public boolean isReversed() {
        return swerveReverseDirection;
    }

    public void changeChassisDrivingDirection() {
        swerveReverseDirection = !swerveReverseDirection;
        // switchLights();
        /*if (!swerveReverseDirection) {
            wheels[0] = frontLeft ;
            wheels[0].setDirection(DcMotor.Direction.REVERSE);

            wheels[1] = frontRight ;
            wheels[1].setDirection(DcMotor.Direction.FORWARD);

            wheels[2] = backLeft ;
            wheels[2].setDirection(DcMotor.Direction.REVERSE);

            wheels[3] = backRight ;
            wheels[3].setDirection(DcMotor.Direction.FORWARD);
        } else {
            wheels[3] = frontLeft ;
            wheels[3].setDirection(DcMotor.Direction.FORWARD);

            wheels[2] = frontRight ;
            wheels[2].setDirection(DcMotor.Direction.REVERSE);

            wheels[1] = backLeft ;
            wheels[1].setDirection(DcMotor.Direction.FORWARD);

            wheels[0] = backRight ;
            wheels[0].setDirection(DcMotor.Direction.REVERSE);
        } */
    }

    public boolean lineDetected(LineColor color) {
        int hueTolerance = 15;              //Tolerance of hues (15 is best)
        double satSensitivity = 0.7;        //How sensitive line detection is (Higher values less sensitive, 0.8 is highest)
        float[] hsvValues = new float[3];
        final double SCALE_FACTOR = 255;

        //final float values[] = hsvValues;
        Color.RGBToHSV((int) (FRColor.red() * SCALE_FACTOR),
                (int) (FRColor.green() * SCALE_FACTOR),
                (int) (FRColor.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[1] >= satSensitivity) {
            if ((hsvValues[0] <= hueTolerance && hsvValues[0] >= 0) || (hsvValues[0] >= 360 - hueTolerance && hsvValues[0] <= 360)) {
                // detect red
                if (color == LineColor.RED) return true;
            } else if (hsvValues[0] >= 220 - hueTolerance && hsvValues[0] <= 220 + hueTolerance) {
                // detect blue
                if (color == LineColor.BLUE) return true;
            }
        }
        return false;
    }

    public void switchLights() {
        if (FRColor == null || FLColor == null)
            return;
        if (isReversed()) {
            FLColor.enableLed(false);
            FRColor.enableLed(false);
        } else {
            FLColor.enableLed(true);
        }
        FRColor.enableLed(true);
    }

    public boolean isSkystone(boolean isRight) {
        if (FRColor == null || FLColor == null)
            return false;

        double distB;
        double addedColors;
        double threshold;
        if (isRight) {
            distB = getDistance(Direction.FRONT_RIGHT);
            addedColors = FRColor.alpha() + FRColor.red() + FRColor.green() + FRColor.blue();
            if (distB <= 7) {
                threshold = -0.206456225 + (1.52138259 / distB);//.221 changed to .206 to keep threshold above skystone argb sum
            } else {
                threshold = 0.006;
            }
            if (addedColors <= threshold) {
                return true;
            } else {
                return false;
            }
        } else {
            distB = getDistance(Direction.FRONT_RIGHT); // DISABLE FRONT_LEFT
            addedColors = FLColor.alpha() + FLColor.red() + FLColor.green() + FLColor.blue();
            if (distB <= 7) {
                threshold = -0.206456225 + (1.52138259 / distB);//.221
            } else {
                threshold = 0.006;
            }
            if (addedColors <= threshold) {
                return true;
            } else {
                return false;
            }
        }
    }

    public void reset() {
        for (WheelAssembly wheel : wheels) wheel.reset(true);
        driveMode = DriveMode.STOP;
        targetHeading = 0;
        switchLights(); // turn on the lights if front direction
    }

    /**
     * Drive in a straight line and maintain heading via IMU
     *
     * @param power   - -1 to 1
     * @param heading - -90 to 90; relative to current robot orientation
     */
    public void driveStraight(double power, double heading) throws InterruptedException {
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        if (driveMode != DriveMode.STRAIGHT) {
            if (driveMode != DriveMode.STOP) {
                // reset motors if they're moving; servos are adjusted below
                for (WheelAssembly wheel : wheels) wheel.reset(false);
            }
            driveMode = DriveMode.STRAIGHT;

            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double[] newServoPositions = new double[4];
            Arrays.fill(newServoPositions, heading);
            changeServoPositions(newServoPositions);

            orientationSensor.enableCorrections(true);
            targetHeading = orientationSensor.getHeading();
        } else {
            // check and correct heading as needed
            double sensorHeading = (orientationSensor == null ? 0 : orientationSensor.getHeading());
            headingDeviation = targetHeading - sensorHeading;
            debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                    targetHeading, sensorHeading, headingDeviation);
            if (Math.abs(headingDeviation) > 0.5) {
                servoCorrection = headingDeviation / 2;
                if (power < 0) {
                    backLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else {
                    frontLeft.servo.adjustPosition(servoCorrection);
                    frontRight.servo.adjustPosition(servoCorrection);
                }
            } else {
                servoCorrection = 0;
                if (power < 0) {
                    backLeft.servo.setPosition(frontLeft.servo.getPosition());
                    backRight.servo.setPosition(frontRight.servo.getPosition());
                } else {
                    frontLeft.servo.setPosition(backLeft.servo.getPosition());
                    frontRight.servo.setPosition(backRight.servo.getPosition());
                }
            }
        }
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(scalePower(power));
    }


    public int calculateOctant(double distance, double heading) {
        int octant = 0;
        if (0 < distance) {
            if (67.5 <= heading)
                octant = 0;
            if (22.5 <= heading && heading < 67.5)
                octant = 1;
            if (-22.5 <= heading && heading < 22.5)
                octant = 2;
            if (-67.5 <= heading && heading < 22.5)
                octant = 3;
            if (heading < -67.5)
                octant = 4;
        } else {
            if (67.5 <= heading)
                octant = 4;
            if (22.5 <= heading && heading < 67.5)
                octant = 5;
            if (-22.5 <= heading && heading < 22.5)
                octant = 6;
            if (-67.5 <= heading && heading < 22.5)
                octant = 7;
            if (heading <= -67.5)
                octant = 0;
        }
        return octant;
    }

    public void applyServoCorrection(int octant, double servoCorrection) {
        if (octant == 0) {
            frontRight.servo.adjustPosition(servoCorrection);
            backRight.servo.adjustPosition(servoCorrection);
        } else if (octant == 1) {
            frontLeft.servo.adjustPosition(servoCorrection);
            backRight.servo.adjustPosition(servoCorrection);
        } else if (octant == 2) {
            frontLeft.servo.adjustPosition(servoCorrection);
            frontRight.servo.adjustPosition(servoCorrection);
        } else if (octant == 3) {
            frontRight.servo.adjustPosition(servoCorrection);
            backLeft.servo.adjustPosition(servoCorrection);
        } else if (octant == 4) {
            frontLeft.servo.adjustPosition(servoCorrection);
            backLeft.servo.adjustPosition(servoCorrection);
        } else if (octant == 5) {
            frontLeft.servo.adjustPosition(servoCorrection);
            backRight.servo.adjustPosition(servoCorrection);
        } else if (octant == 6) {
            backLeft.servo.adjustPosition(servoCorrection);
            backRight.servo.adjustPosition(servoCorrection);
        } else {
            frontRight.servo.adjustPosition(servoCorrection);
            backLeft.servo.adjustPosition(servoCorrection);
        }
    }

    public static double square(double x) {
        return x * x;
    }

    //using the indicated absolute power to drive a certain distance at a certain heading
    public void driveStraightAuto(double power, double cm, double heading, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }
        boolean shouldApplyIMU = false;
        if (Math.abs(cm) < 20) shouldApplyIMU = false;
        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        //octant will determine which wheels to use to adjust heading deviation
        int octant = calculateOctant(distance, heading);

        if (Thread.interrupted()) return;

        if (distance < 0) {
            power = -power;
            distance = -distance;
        }

        //motor settings
        driveMode = DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            if (Thread.interrupted()) return;
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        //servo settings
        double[] newServoPositions = new double[4];
        Arrays.fill(newServoPositions, heading);
        changeServoPositions(newServoPositions);
        if (Thread.interrupted()) return;

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering wheels
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);
        if (Thread.interrupted()) return;

        //record time
        long iniTime = System.currentTimeMillis();
        double slowDownPercent = .8;
        if (power > .7 && cm < 100)
            slowDownPercent = .7;
        else if (power > .5 && cm < 100)
            slowDownPercent = .75;

        //waiting loop
        while (true) {
            if (shouldApplyIMU) {
                // check and correct heading as needed
                double sensorHeading = orientationSensor.getHeading();
                headingDeviation = targetHeading - sensorHeading;
                debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                        targetHeading, sensorHeading, headingDeviation);
                if (Math.abs(headingDeviation) > .5) {
                    servoCorrection = headingDeviation / 3;
                    applyServoCorrection(octant, servoCorrection);

                } else {
                    servoCorrection = 0;
                    if (power < 0) {
                        backLeft.servo.setPosition(frontLeft.servo.getPosition());
                        backRight.servo.setPosition(frontRight.servo.getPosition());
                    } else {
                        frontLeft.servo.setPosition(backLeft.servo.getPosition());
                        frontRight.servo.setPosition(backRight.servo.getPosition());
                    }
                }
            }
            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                if (Thread.interrupted()) return;
                maxTraveled = Math.abs(Math.max(maxTraveled, wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }
            double traveledPercent = maxTraveled / Math.abs(distance);

            if (traveledPercent > slowDownPercent) {
                if (traveledPercent >= 0.99)
                    break;
                double apower = Math.abs(power);

                double pow = .25 * minPower + .75 * apower;
                if (traveledPercent < .25 + .75 * slowDownPercent)
                    pow = .5 * minPower + .5 * apower;
                else if (traveledPercent < .5 + .5 * slowDownPercent)
                    pow = .75 * minPower + .25 * apower;
                else if (traveledPercent < .75 + .25 * slowDownPercent) pow = minPower;

                if (pow < minPower) pow = minPower;
                pow = pow * Math.signum(power);

                for (WheelAssembly wheel : wheels) wheel.motor.setPower(pow);
                //tl.addLine("in the last 20%");
                //tl.addData("power output %f", pow);
            }

            //if (distance - maxTraveled < 10)
            //break;
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.interrupted()) return;
            TaskManager.processTasks();
            // yield handler
            // this.core.yield();
        }
        if (Thread.interrupted()) return;
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
        driveMode = DriveMode.STOP;
    }

    public void driveStraightAutoRunToPosition(double power, double cm, double heading, int timeout) throws InterruptedException {
        driveStraightAutoRunToPosition(power, cm, heading, 0.0, timeout);
    }

    public void driveAuto(double power, double cm, double heading, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        double overshoot = 1;
        if (cm > 0) {
            if (cm <= 10) {
                overshoot = 1.4;
            } else if (cm <= 20) {
                overshoot = 1.2;
            } else if ((cm <= 35)) {
                overshoot = 1.15;
            } else if ((cm <= 100)) {
                overshoot = 1 + (power - .2) / 2.5;
            } else if (power >= .7 && cm < 150) {
                overshoot = 1.1;
            }
        } else {
            if (Math.abs(cm) <= 10) {
                overshoot = 1.3;
            } else if (Math.abs(cm) <= 20) {
                overshoot = 1.2;
            } else if ((Math.abs(cm) <= 35)) {
                overshoot = 1.15;
            } else if ((Math.abs(cm) <= 100)) {
                overshoot = 1 + (power - .2) / 3.1;
            } else if (power >= .7 && Math.abs(cm) < 150) {
                overshoot = 1.1;
            }
        }
        if (Thread.interrupted()) return;
        driveStraightAuto(power, cm / overshoot, heading, timeout);
    }

    public void driveStraightAutoRunToPosition(double power, double cm, double heading, double beginTaskPercent, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }

        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        //octant will determine which wheels to use to adjust heading deviation
        int octant = calculateOctant(distance, heading);

        //motor settings
        driveMode = DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].motor.setTargetPosition(wheels[i].motor.getCurrentPosition() + (int) distance);
            startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //servo settings
        double[] newServoPositions = new double[4];
        Arrays.fill(newServoPositions, heading);
        changeServoPositions(newServoPositions);

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering wheels
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);

        //record time
        long iniTime = System.currentTimeMillis();

        //waiting loop
        while (wheels[0].motor.isBusy() && wheels[1].motor.isBusy() && wheels[2].motor.isBusy() && wheels[3].motor.isBusy()) {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;
            debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                    targetHeading, sensorHeading, headingDeviation);
            if (Math.abs(headingDeviation) > 0.5) {//was 0.5
                servoCorrection = headingDeviation / 2.0;//was 3.0
                applyServoCorrection(octant, servoCorrection);
            } else {
                servoCorrection = 0;
//                if (distance < 0) {
//                    backLeft.servo.setPosition(frontLeft.servo.getPosition());
//                    backRight.servo.setPosition(frontRight.servo.getPosition());
//                } else {
//                    frontLeft.servo.setPosition(backLeft.servo.getPosition());
//                    frontRight.servo.setPosition(backRight.servo.getPosition());
//                }
            }
            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                maxTraveled = Math.max(maxTraveled, Math.abs(wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }

            double traveledPercent = maxTraveled / Math.abs(distance);
            if (traveledPercent > bufferPercentage) {
                if (traveledPercent >= 0.99)
                    break;
                double pow = (power - minPower) * square(1 - square((traveledPercent - cutoffPercent) / (1 - cutoffPercent))) + minPower;
                //(power - minPower)*(1-(traveledPercent - cutoffPercent)/(1-cutoffPercent) * (traveledPercent - cutoffPercent)/(1-cutoffPercent))* (1-(traveledPercent - cutoffPercent)/(1-cutoffPercent) * (traveledPercent - .8)/(1-.8)) + minPower;
                for (WheelAssembly wheel : wheels) wheel.motor.setPower(pow);
                //tl.addLine("in the last 20%");
                //tl.addData("power output %f", pow);
            } else {
                //tl.addLine("in the first 80%");
            }
            //tl.update();
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.interrupted())
                return;

            //take care of other business
            if (traveledPercent > beginTaskPercent) {
                TaskManager.processTasks();
            }
            // yield handler
            //this.core.yield();
        }

        for (WheelAssembly wheel : wheels) {
            wheel.motor.setPower(0.0);
            wheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveMode = DriveMode.STOP;
    }

    public void driveStraightAutoRunToPositionNoIMU(double power, double cm, double heading, int timeout) throws InterruptedException {
        universalDriveStraight(power, cm, heading, timeout, false);
    }

    public void universalDriveStraight(double power, double cm, double heading, int timeout, boolean imuCorrection) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }

        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        //motor settings
        driveMode = DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (cm > 20)
                wheels[i].motor.setTargetPosition(wheels[i].motor.getCurrentPosition() + (int) distance);
            startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(cm > 20 ? DcMotor.RunMode.RUN_TO_POSITION : DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //servo settings
        double[] newServoPositions = new double[4];
        Arrays.fill(newServoPositions, heading);
        changeServoPositions(newServoPositions);

        //imu initialization
        if (orientationSensor != null) {
            orientationSensor.enableCorrections(true);
            targetHeading = orientationSensor.getHeading();
        }
        //octant will determine which wheels to use to adjust heading deviation
        int octant = calculateOctant(distance, heading);

        //start powering wheels
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);

        //record time
        long iniTime = System.currentTimeMillis();

        //waiting loop
        while (true) {
            if (cm > 20 || !(wheels[0].motor.isBusy() && wheels[1].motor.isBusy() && wheels[2].motor.isBusy() && wheels[3].motor.isBusy())) {
                break;//when using RUN_TO_POSITION and one motor has reached target
            }

            if (imuCorrection && (orientationSensor != null)) {
                double sensorHeading = orientationSensor.getHeading();
                headingDeviation = targetHeading - sensorHeading;
                if (Math.abs(headingDeviation) > 0.5) {//was 0.5
                    servoCorrection = headingDeviation / 2.0;//was 3.0
                    applyServoCorrection(octant, servoCorrection);
                } else {
                    servoCorrection = 0;
                }
            }

            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                maxTraveled = Math.max(maxTraveled, Math.abs(wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }

            if (maxTraveled / Math.abs(distance) > bufferPercentage) {
                double traveledPercent = maxTraveled / Math.abs(distance);
                if (traveledPercent >= 0.99) //determine if target distance is reached
                    break;
                double pow = (power - minPower) * square(1 - square((traveledPercent - cutoffPercent) / (1 - cutoffPercent))) + minPower;
                pow = Math.max(pow, minPower);
                for (WheelAssembly wheel : wheels) wheel.motor.setPower(pow);
            }

            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.interrupted())
                return;

            //take care of other business
            TaskManager.processTasks();

        }

        for (WheelAssembly wheel : wheels) {
            wheel.motor.setPower(0.0);
            wheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //tl.addData("number of loops %d",loop);
        //tl.addData("ini encoder %d",iniEncoder);
        //tl.addData("final encoder of loops %d",finalEncoder);
        //tl.addData("total loop time %d",finalTime-iniTime);
        //tl.update();
        driveMode = DriveMode.STOP;
    }

    public void driveStraightAutoRunToWall(double power, double cm, Direction dir, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;

        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }


        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        //octant will determine which wheels to use to adjust heading deviation
        int octant = 0;
        switch (dir) {
            case RIGHT:
                octant = 0;
                break;
            case LEFT:
                octant = 4;
                break;
            case FRONT:
                octant = 2;
                break;
            case BACK:
                octant = 6;
        }


//        tl.addData("octant",octant);
//        tl.update();
//        sleep(3000);

        //motor settings
        driveMode = DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].motor.setTargetPosition(wheels[i].motor.getCurrentPosition() + (int) distance);
            startingCount[i] = wheels[i].motor.getCurrentPosition();
        }


        //servo settings
        double[] newServoPositions = new double[4];
//        Arrays.fill(newServoPositions, heading);
        changeServoPositions(newServoPositions);

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering wheels
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);

        //record time
        long iniTime = System.currentTimeMillis();

        //waiting loop
        while (wheels[0].motor.isBusy() && wheels[1].motor.isBusy() && wheels[2].motor.isBusy() && wheels[3].motor.isBusy()) {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;
            debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                    targetHeading, sensorHeading, headingDeviation);
            if (Math.abs(headingDeviation) > 1.0) {//was 0.5
                servoCorrection = headingDeviation / 4.0;//was 3.0
                if (octant == 0) {  // right
                    frontRight.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else if (octant == 1) { // front right around 45
                    frontLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else if (octant == 2) { // forward
                    frontLeft.servo.adjustPosition(servoCorrection);
                    frontRight.servo.adjustPosition(servoCorrection);
                } else if (octant == 3) { // front left around 45
                    frontRight.servo.adjustPosition(servoCorrection);
                    backLeft.servo.adjustPosition(servoCorrection);
                } else if (octant == 4) { // left
                    frontLeft.servo.adjustPosition(servoCorrection);
                    backLeft.servo.adjustPosition(servoCorrection);
                } else if (octant == 5) { // back left around 45
                    frontLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else if (octant == 6) { // backward
                    backLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else { // back right around 45
                    frontRight.servo.adjustPosition(servoCorrection);
                    backLeft.servo.adjustPosition(servoCorrection);
                }
            } else {
                servoCorrection = 0;
//                if (distance < 0) {
//                    backLeft.servo.setPosition(frontLeft.servo.getPosition());
//                    backRight.servo.setPosition(frontRight.servo.getPosition());
//                } else {
//                    frontLeft.servo.setPosition(backLeft.servo.getPosition());
//                    frontRight.servo.setPosition(backRight.servo.getPosition());
//                }
            }
            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                maxTraveled = Math.max(maxTraveled, Math.abs(wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }

            if (maxTraveled / Math.abs(distance) > bufferPercentage) {
                double traveledPercent = maxTraveled / Math.abs(distance);
                if (traveledPercent >= 1.0)
                    break;
                double pow = (power - minPower) * Math.pow(1 - Math.pow((traveledPercent - cutoffPercent) / (1 - cutoffPercent), 2), 2) + minPower;
                //(power - minPower)*(1-(traveledPercent - cutoffPercent)/(1-cutoffPercent) * (traveledPercent - cutoffPercent)/(1-cutoffPercent))* (1-(traveledPercent - cutoffPercent)/(1-cutoffPercent) * (traveledPercent - .8)/(1-.8)) + minPower;
                for (WheelAssembly wheel : wheels) wheel.motor.setPower(pow);
                //tl.addLine("in the last 20%");
                //tl.addData("power output %f", pow);
            } else {
                //tl.addLine("in the first 80%");
            }
            //tl.update();
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.interrupted())
                return;

            //take care of other business
            TaskManager.processTasks();
            // yield handler
            //this.core.yield();
        }

        for (WheelAssembly wheel : wheels) {
            wheel.motor.setPower(0.0);
            wheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //tl.addData("number of loops %d",loop);
        //tl.addData("ini encoder %d",iniEncoder);
        //tl.addData("final encoder of loops %d",finalEncoder);
        //tl.addData("total loop time %d",finalTime-iniTime);
        //tl.update();
        driveMode = DriveMode.STOP;
    }


    public enum Direction {
        FRONT, LEFT, RIGHT, BACK, FRONT_LEFT, FRONT_RIGHT, LEFT_HI, RIGHT_HI, RIGHT_FRONT, RIGHT_BACK;
    }

    public double getCurHeading() {
        return curHeading;
    }

    /**
     * Drive using currently specified power and heading values
     *
     * @param power     -1 to 1
     * @param heading   -90 to 90; relative to current robot orientation
     * @param allWheels <code>true</code> to use all 4 wheels,
     *                  <code>false</code> to use front wheels only
     */
    public void driveAndSteer(double power, double heading, boolean allWheels) throws InterruptedException {
        double leftPower = power;
        double rightPower = power;
        debug("driveSteer(pwr: %.3f, head: %.1f)", power, heading);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }
        if (driveMode != DriveMode.STEER) {
            if (driveMode != DriveMode.STOP) reset();
            driveMode = DriveMode.STEER;
            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (Math.abs(power) > 0) {
            // only adjust servo positions if power is applied
            double[] newServoPositions = new double[4];

            if (allWheels) {
                if (Math.abs(heading) == 90) {
                    // check whether all servos are already at 90 (or -90) degrees
                    boolean samePosition = (frontLeft.servo.getPosition() == frontRight.servo.getPosition())
                            && (frontLeft.servo.getPosition() == backLeft.servo.getPosition())
                            && (frontLeft.servo.getPosition() == backRight.servo.getPosition())
                            && (Math.abs(frontLeft.servo.getPosition()) == 90);
                    // keep wheels pointed sideways and invert the power if needed
                    if (samePosition) {
                        power *= heading == frontLeft.servo.getPosition() ? 1 : -1;
                        heading = frontLeft.servo.getPosition();
                    }
                }
                Arrays.fill(newServoPositions, heading);
                leftPower = power;
                rightPower = power;
            } else {
                // complement to angle between Y axis and line from the center of chassis,
                //  which is assumed to be at (0, 0) to the center of the front right wheel
                double minTurnRadius = 20; //Somewhat arbitrarily determined minimum turning radius for car mode, in inches
                double maxTurnRadius = 100; //Maximum turning radius for car mode, carried over from last year, in inches
                // double radius = Math.signum(heading)*(maxTurnRadius - ((maxTurnRadius - minTurnRadius) * Math.abs(heading/90))); //Converts a heading from -90 to 90 to a radius from -100 to 100
                double radius = Math.signum(heading) * Math.signum(power) * (maxTurnRadius - ((maxTurnRadius - minTurnRadius) * Math.abs(heading / 90))); //Converts a heading from -90 to 90 to a radius from -100 to 100
                double innerAngle = Math.atan(wheelBase / (2 * radius - track)) * 180 / Math.PI; //Misnomer from first coding, negative radius will flip this from inner to outer
                double outerAngle = Math.atan(wheelBase / (2 * radius + track)) * 180 / Math.PI; //Cont. from above: inner refers to right side, outer refers to left side
                double innerRadius = Math.hypot(0.5 * wheelBase, radius - 0.5 * track);
                double outerRadius = Math.hypot(0.5 * wheelBase, radius + 0.5 * track);
                double innerPower = power * (innerRadius / outerRadius);
                double outerPower = power;
                // front left and right
                newServoPositions[0] = outerAngle;
                newServoPositions[1] = innerAngle;
                // back left and right
                newServoPositions[2] = -1 * outerAngle;
                newServoPositions[3] = -1 * innerAngle;
                // left and right powers
                leftPower = outerPower;
                rightPower = innerPower;
            }
            changeServoPositions(newServoPositions);
            curHeading = heading;
        }
        wheels[0].motor.setPower(scalePower(leftPower));
        wheels[1].motor.setPower(scalePower(rightPower));
        wheels[2].motor.setPower(scalePower(leftPower));
        wheels[3].motor.setPower(scalePower(rightPower));
    }

    public void toggleTankDrive() {
        isTankDrive = !isTankDrive;
    }

    public void tankDrive(double leftPower, double rightPower) throws InterruptedException {
        if (leftPower < -1 || leftPower > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (rightPower < -1 || rightPower > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (driveMode != DriveMode.TANK) {
            driveMode = DriveMode.TANK;
            double[] newServoPositions = new double[4];
            Arrays.fill(newServoPositions, 0);
            changeServoPositions(newServoPositions);
            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        wheels[0].motor.setPower(scalePower(leftPower));
        wheels[1].motor.setPower(scalePower(rightPower));
        wheels[2].motor.setPower(scalePower(leftPower));
        wheels[3].motor.setPower(scalePower(rightPower));
    }

    public void orbit(double power, double curvature, boolean orbitBack) throws InterruptedException {
        double leftPower;
        double rightPower;
        double radius = 6.5 * (2.0 - Math.abs(curvature));
        boolean shouldReverse = orbitBack;

        if (isReversed()) {
            shouldReverse = !shouldReverse;
            power *= -1;
        }

        double thetaF = (Math.atan(radius / (0.5 * track))) * (180 / Math.PI);
        double thetaB = (Math.atan((radius + wheelBase) / (0.5 * track))) * (180 / Math.PI);
        double SERVO_FL_ORBIT_POSITION = (-thetaF);
        double SERVO_FR_ORBIT_POSITION = (thetaF);
        double SERVO_BL_ORBIT_POSITION = (-thetaB);
        double SERVO_BR_ORBIT_POSITION = (thetaB);

        if (shouldReverse) {
            SERVO_FL_ORBIT_POSITION = thetaB;
            SERVO_FR_ORBIT_POSITION = -thetaB;
            SERVO_BL_ORBIT_POSITION = thetaF;
            SERVO_BR_ORBIT_POSITION = -thetaF;
        }

        debug("orbit(pwr: %.3f, theta(F/B): %.1f/%.1f)", power, thetaF, thetaB);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }

        if (driveMode != DriveMode.STEER) {
            if (driveMode != DriveMode.STOP) reset();
            driveMode = DriveMode.STEER;
            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (Math.abs(power) > 0) {
            // only adjust servo positions if power is applied
            double[] newServoPositions = new double[4];
            newServoPositions[0] = SERVO_FL_ORBIT_POSITION;
            newServoPositions[1] = SERVO_FR_ORBIT_POSITION;
            newServoPositions[2] = SERVO_BL_ORBIT_POSITION;
            newServoPositions[3] = SERVO_BR_ORBIT_POSITION;
            changeServoPositions(newServoPositions);
        }

        leftPower = -power;
        rightPower = power;

        wheels[0].motor.setPower(scalePower(leftPower));
        wheels[1].motor.setPower(scalePower(rightPower));
        wheels[2].motor.setPower(scalePower(leftPower));
        wheels[3].motor.setPower(scalePower(rightPower));
    }

    public void changeStopBehavior(boolean isBreak) {
        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setZeroPowerBehavior(isBreak ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    public double driveStraightSec(double power, double sec, boolean noStop) throws InterruptedException {
        if (Thread.interrupted()) return 0;
        double[] startingCount = new double[4];
        for (int i = 0; i < 4; i++) {
            startingCount[i] = wheels[i].motor.getCurrentPosition();
            //    wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        long startTime = System.currentTimeMillis();
        driveAndSteer(power, 0, true);
        sleep((long) (sec * 1000.0));
        if (!noStop) {
            driveAndSteer(0, 0, true);
        }
        double ave = 0;
        for (int i = 0; i < 4; i++) {
            startingCount[i] = (wheels[i].motor.getCurrentPosition() - startingCount[i]) / sec;
            ave += startingCount[i];
        }
        return (ave / 4.0); // return average count per second
    }


    /**
     * Rotate in place using currently specified power
     */
    public void rotate(double power) throws InterruptedException {
        debug("rotate(pwr: %.3f)", power);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (driveMode != DriveMode.ROTATE) {
            if (driveMode != DriveMode.STOP) {
                // reset motors if they're moving; servos are adjusted below
                for (WheelAssembly wheel : wheels) wheel.reset(false);
            }
            driveMode = DriveMode.ROTATE;

            for (WheelAssembly wheel : wheels) {
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // angle between Y axis and line from the center of chassis,
            //  which is assumed to be at (0, 0) to the center of the front right wheel
            double angle = Math.atan2(track, wheelBase) / Math.PI * 180;
            double[] newServoPositions = new double[4];
            // front left and back right
            newServoPositions[0] = newServoPositions[3] = angle;
            // front right and back left
            newServoPositions[1] = newServoPositions[2] = -1 * angle;
            changeServoPositions(newServoPositions);
        }

        frontLeft.motor.setPower(useScalePower ? scalePower(power) : power);
        frontRight.motor.setPower(-1 * (useScalePower ? scalePower(power) : power));
        backLeft.motor.setPower(useScalePower ? scalePower(power) : power);
        backRight.motor.setPower(-1 * (useScalePower ? scalePower(power) : power));
    }

    public void stop() {
        if (frontLeft != null && frontLeft.motor != null)
            frontLeft.motor.setPower(0);
        if (frontRight != null && frontRight.motor != null)
            frontRight.motor.setPower(0);
        if (backLeft != null && backLeft.motor != null)
            backLeft.motor.setPower(0);
        if (backRight != null && backRight.motor != null)
            backRight.motor.setPower(0);
    }

    @Deprecated
    public void rotateDegree(double power, double deltaD) throws InterruptedException {
        double iniHeading = orientationSensor.getHeading();
        double finalHeading = iniHeading + deltaD;
        if (finalHeading > 180)
            finalHeading -= 360;
        else if (finalHeading < -180)
            finalHeading += 360;
        rotateTo(power, finalHeading);
    }

    @Deprecated
    public void rotateToOld(double power, double finalHeading) throws InterruptedException {
        rotateTo(power, finalHeading, 3000);
    }

    @Deprecated
    public void rotateToOld(double power, double finalHeading, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        if (power < 0.3) {//when power is small, use a flat power output
            rawRotateTo(power, finalHeading, true, timeout);
            return;
        }
        //when power is big, use a piecewise power output
        double iniHeading = orientationSensor.getHeading();
        double iniHeading_P = -iniHeading;
        double finalHeading_P = -finalHeading;
        double absDiff = min(abs(finalHeading - iniHeading_P), 180 - abs(finalHeading - iniHeading_P));
        double firstTarget;
        if (cos(iniHeading_P * degreeToRad) * sin(finalHeading_P * degreeToRad) - cos(finalHeading_P * degreeToRad) * sin(iniHeading_P * degreeToRad) >= 0) {//rotating ccw
            firstTarget = iniHeading - 0.8 * absDiff;
        } else {
            firstTarget = iniHeading + 0.8 * absDiff;
        }
        //make sure first target stay in [-180,+180] range
        if (firstTarget > 180) firstTarget -= 360;
        if (firstTarget < -180) firstTarget += 360;
        //for debug use
        tl.addData("iniHeading", iniHeading);
        tl.addData("finalHeading", finalHeading);
        tl.addData("firstTarget", firstTarget);
        tl.update();
        rawRotateTo(power, firstTarget, true, timeout);
        //sleep(100);
        if (Thread.interrupted()) return;
        rawRotateTo(chassisAligmentPower, finalHeading, false, 1000);
    }

    static final double degreeToRad = PI / 180;
    static final double radToDegree = 180 / PI;

    public void rotateTo(double power, double finalHeading) throws InterruptedException {
        rotateTo(power, finalHeading, 4000);
    }

    public void rotateTo(double power, double finalHeading, int timeout) throws InterruptedException {
        rotateTo(power, finalHeading, timeout, true,true);
    }

    public void rotateTo(double power, double finalHeading, int timeout, boolean changePower, boolean finalCorrection) throws InterruptedException {
        if (Thread.interrupted()) return;
        if (power <= chassisAligmentPower) {//was 0.3
            rawRotateTo(power, finalHeading, false, timeout);//was power
            if (Thread.interrupted()) return;
            if (power > chassisAligmentPower)
                rawRotateTo(chassisAligmentPower, finalHeading, false, timeout);
            return;
        }
        double iniHeading = orientationSensor.getHeading();
        double iniAbsDiff = abs(finalHeading - iniHeading) > 180 ? 360 - abs(finalHeading - iniHeading) : abs(finalHeading - iniHeading);
        if (iniAbsDiff < 0.5)//if within 0.5 degree of target, don't rotate
            return;

        int direction;
        if (cross(-iniHeading, -finalHeading) >= 0) {//revert sign and take cross product
            direction = -1;//rotating ccw
        } else {
            direction = +1;//rotating cw
        }
        if (Thread.interrupted()) return;
        double lowPowerDegree = 8 + (power - 0.3) * 70;
        //break on reaching the target
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        driveMode = DriveMode.STOP;
        useScalePower = false;
        //power the wheels
        if (Thread.interrupted()) return;
        rotate(direction * power);
        double currentHeading;
        double crossProduct;
        double currentAbsDiff;
        boolean lowerPowerApplied = false;
        long iniTime = System.currentTimeMillis();
        int loop = 0;
        while (true) {
            if (Thread.interrupted()) return;
            currentHeading = orientationSensor.getHeading();
//            info("RotateTo-%.1f, heading =%.3f, pw=%.2f(%s)", finalHeading,currentHeading,power,(lowerPowerApplied?"low":"hi"));
            crossProduct = cross(-currentHeading, -finalHeading);
            //break if target reached or exceeded
            if (direction == -1) {//rotating ccw
                if (crossProduct <= 0) break;
            } else {//rotating cw
                if (crossProduct >= 0) break;
            }
            currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
            if (changePower && !lowerPowerApplied && currentAbsDiff <= lowPowerDegree) {//damp power to 0.22 if in last 40%, (currentAbsDiff / iniAbsDiff < 0.40)
                if (Thread.interrupted()) return;
                rotate(0.0);
                sleep(100);
                if (Thread.interrupted()) return;
                rotate(direction * chassisAligmentPower);
                lowerPowerApplied = true;
            }
            if (currentAbsDiff / iniAbsDiff < 0.20 && abs(crossProduct) * radToDegree < 1.0)//assume sinx=x, stop 1 degree early
                break;//stop if really close to target
            if (Thread.interrupted()) break;
            if (System.currentTimeMillis() - iniTime > timeout) break;
            TaskManager.processTasks();
            loop++;
        }
        if (Thread.interrupted()) return;
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
        if (!finalCorrection) {
            driveMode = DriveMode.STOP;
            useScalePower = true;
            return;
        }
        if (Thread.interrupted()) return;
        sleep(100);
        //**************Check for overshoot and correction**************
        currentHeading = orientationSensor.getHeading();
        currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
        if ((currentAbsDiff > 2.0) && !Thread.interrupted()) {
            rawRotateTo(0.22, finalHeading, false, 500);
        }
        if (Thread.interrupted()) return;
        //**************End correction**************
        driveMode = DriveMode.STOP;
        useScalePower = true;
//        tl.addData("iteration",loop);
//        tl.update();
//        sleep(3000);
    }

    //final heading needs to be with in range(-180,180]
    public void rawRotateTo(double power, double finalHeading, boolean stopEarly, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        debug("iniHeading: %.1f, deltaD: %.1f)", iniHeading, deltaD);
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;
        //resolve the issue with +-180 mark
        if (Math.abs(deltaD) > 180) {
            finalHeading = finalHeading + (deltaD > 0 ? -360 : +360);
            deltaD = 360 - Math.abs(deltaD);
            deltaD = -deltaD;
            debug("Adjusted finalHeading: %.1f, deltaD: %.1f)", finalHeading, deltaD);
        }
        //break on reaching the target
        if (Thread.interrupted()) return;
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        rotate(Math.signum(deltaD) * power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            if (Thread.interrupted()) return;
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? power * 10.0 : 0.5)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;
            //timeout, break. default timeout: 3s
            if (System.currentTimeMillis() - iniTime > timeout) break;
            //stop pressed, break
            if (Thread.interrupted()) return;
            lastReading = currentHeading;
//            sleep(0);
            // yield handler
            TaskManager.processTasks();
            this.core.yield();
        }
        if (Thread.interrupted()) return;
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
        driveMode = DriveMode.STOP;
        useScalePower = true;
    }

    //cross two unit vectors whose argument angle is given in degree
    public static double cross(double theta, double phi) {
        return cos(theta * degreeToRad) * sin(phi * degreeToRad) - sin(theta * degreeToRad) * cos(phi * degreeToRad);
    }

    public void rawRotateToSimplified(double power, double finalHeading, boolean ccw, boolean stopEarly, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        debug("iniHeading: %.1f, deltaD: %.1f)", iniHeading, deltaD);
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;

        //break on reaching the target
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        rotate(ccw ? -power : power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? power * 10.0 : 0.5)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;
            //timeout, break. default timeout: 3s
            if (System.currentTimeMillis() - iniTime > timeout) break;
            //stop pressed, break
            if (Thread.interrupted()) return;
            lastReading = currentHeading;
//            sleep(0);
            // yield handler
            TaskManager.processTasks();
            this.core.yield();
        }
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
        driveMode = DriveMode.STOP;
        useScalePower = true;
    }


    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
    }

    /**
     * Adjusts servo positions and waits for them to turn
     *
     * @param newPositions new servo positions matching wheel assembly order:
     *                     front left, front right, back left, back right
     */
    private void changeServoPositions(double[] newPositions) throws InterruptedException {
        if (Thread.interrupted()) return;
        double maxServoAdjustment = 0;
        for (int index = 0; index < newPositions.length; index++) {
            double servoAdjustment = Math.abs(newPositions[index] - wheels[index].servo.getPosition());
            maxServoAdjustment = Math.max(maxServoAdjustment, servoAdjustment);
            wheels[index].servo.setPosition(newPositions[index]);
        }
        if (!Thread.interrupted())
            sleep((int) Math.round(3 * maxServoAdjustment));
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.interrupted()) return;
        tl = telemetry;
        Telemetry.Line line = telemetry.addLine();
        line.addData("DriveMode", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s", (isTankDrive() ? "Tank" : "Normal"));
            }
        });
        line.addData("Pwr/Head/Scale", new Func<String>() {
            @Override
            public String value() {
                return String.format("%.2f/%.1f/%.1f/%s", frontLeft.motor.getPower(), curHeading, getDefaultScale(), (isReversed() ? "(R)" : "(F)"));
            }
        });
        if (frontLeft.motor != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return frontLeft.motor.getCurrentPosition();
                }
            });
        }
        if (frontRight.motor != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return frontRight.motor.getCurrentPosition();
                }
            });
        }
        if (backLeft.motor != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return backLeft.motor.getCurrentPosition();
                }
            });
        }
        if (backRight.motor != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return backRight.motor.getCurrentPosition();
                }
            });
        }

        //set up imu telemetry
        if (orientationSensor != null && setImuTelemetry) {
            line.addData("imuC", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

        //set up range sensor telemetry
        if (setRangeSensorTelemetry) {
            if (rightRangeSensor != null) {
                line.addData("rangeR", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rightRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (rightHiRangeSensor != null) {
                line.addData("rangeRH", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rightHiRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (leftRangeSensor != null) {
                line.addData("rangeL", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return leftRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (leftHiRangeSensor != null) {
                line.addData("rangeLH", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return leftHiRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (frontLeftRangeSensor != null) {
                line.addData("rangeFL", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return frontLeftRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (frontRightRangeSensor != null) {
                line.addData("rangeFR", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return frontRightRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
            if (backRangeSensor != null) {
                line.addData("rangeB", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return backRangeSensor.getDistance(DistanceUnit.CM);
                    }
                });
            }
        }

//        if (FRColor!=null) {
//            line.addData("blue color = ", "%s", new Func<String>() {
//                @Override
//                public String value() {
//                    return (lineDetected(LineColor.BLUE)?"T":"F");
//                }
//            });
//            line.addData("red color = ", "%s", new Func<String>() {
//                @Override
//                public String value() {
//                    return (lineDetected(LineColor.RED)?"T":"F");
//                }
//            });
//        }
        if (showColor) {
            if (FLColor != null) {
                line.addData("Skystone-FL = ", "%s", new Func<String>() {
                    @Override
                    public String value() {
                        return (isSkystone(true) ? "T" : "F");
                    }
                });
                line.addData("FLColor-Sum = ", "%.3f", new Func<Double>() {
                    @Override
                    public Double value() {
                        double addedColors = FLColor.alpha() + FLColor.red() + FLColor.green() + FLColor.blue();
                        return addedColors;
                    }
                });
            }

            if (FRColor != null) {
                line.addData("Skystone-FR = ", "%s", new Func<String>() {
                    @Override
                    public String value() {
                        return (isSkystone(false) ? "T" : "F");
                    }
                });
                line.addData("FRColor-Sum = ", "%.3f", new Func<Double>() {
                    @Override
                    public Double value() {
                        double addedColors = FRColor.alpha() + FRColor.red() + FRColor.green() + FRColor.blue();
                        return addedColors;
                    }
                });
            }
            if (FLDistance != null) {
                line.addData("FL-dist = ", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return getDistanceColor(FLDistance);
                    }
                });
            }
            if (FRDistance != null) {
                line.addData("FR-dist = ", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return getDistanceColor(FRDistance);
                    }
                });
            }
            if (FLDistance != null && FRDistance != null) {
                line.addData("SkystoneLOC(Blue) = ", "%s", new Func<String>() {
                    @Override
                    public String value() {
                        ToboSigma.SkystoneLocation loc = getSkystonePositionColor(false);
                        return loc.toString();
                    }
                });
                line.addData("SkystoneLOC(Red) = ", "%s", new Func<String>() {
                    @Override
                    public String value() {
                        ToboSigma.SkystoneLocation loc = getSkystonePositionColor(true);
                        return loc.toString();
                    }
                });
            }
            line.addData("stoneCollected = ", "%s", new Func<String>() {
                @Override
                public String value() {
                    return (stoneCollected() ? "T" : "F");
                }
            });
        }
        telemetry.addLine().addData("M", new Func<String>() {
            @Override
            public String value() {
                return driveMode.name();
            }
        }).addData("Head", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f (%+.2f)", targetHeading, headingDeviation);
            }
        }).addData("Adj", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f", servoCorrection);
            }
        });
        line = telemetry.addLine("Srv: ");
        for (WheelAssembly wheel : wheels) {
            final AdjustableServo servo = wheel.servo;
            line.addData(wheel.position, "%+.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return servo.getPosition();
                }
            });
        }

    }

    public void resetOrientation() {
        orientationSensor.reset();
    }

    public boolean hasRollStabalized(int inputIndex, double minDiff) {
        return orientationSensor.hasRollStabalized(inputIndex, minDiff);
    }

    final class WheelAssembly {
        AdjustableServo servo;
        DcMotor motor;
        String position;

        WheelAssembly(Configuration configuration, String position, DcMotor.Direction direction) {
            // abbreviate position, leaving only capital letters
            StringBuilder sb = new StringBuilder(position);
            int index = 0;
            while (index < sb.length()) {
                if (Character.isUpperCase(sb.charAt(index))) {
                    index++;
                } else {
                    sb.deleteCharAt(index);
                }
            }
            this.position = sb.toString();

            servo = new AdjustableServo().configureLogging(
                    logTag + ":servo" + this.position, logLevel
            );
            servo.configure(configuration.getHardwareMap(), "servo" + position);
            configuration.register(servo);

            // motor = configuration.getHardwareMap().get(DcMotor.class, "motor" + position);
            motor = configuration.getHardwareMap().tryGet(DcMotor.class, "motor" + position);
            if (motor != null) motor.setDirection(direction);
        }

        void reset(boolean resetServo) {
            if (motor == null) return;
            motor.setPower(0.0d);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (resetServo) servo.reset();
        }

        void setDirection(DcMotor.Direction direction) {
            if (motor != null) motor.setDirection(direction);
        }
    }

    public double getDistanceColor(DistanceSensor sen) {
        if (Thread.interrupted()) return 0;
        double dist = sen.getDistance(DistanceUnit.CM);
        if (dist == (double) (Double.MAX_VALUE) || dist > 128.0 || dist <= 0.0 || dist == (double) distanceOutOfRange || dist == (double) (DistanceUnit.infinity))
            dist = 1024.0;
        else if (Double.isNaN(dist))
            dist = 1024.0;

        return dist;
    }

    public ToboSigma.SkystoneLocation getSkystonePositionColor(boolean redSide) {
        if (Thread.interrupted()) return ToboSigma.SkystoneLocation.UNKNOWN;
        double distL = getDistanceColor(FLDistance);
        double distR = getDistanceColor(FRDistance);

        if (!redSide) // blue side
        {
            if (Math.abs(distL - distR) <= 10)
                return ToboSigma.SkystoneLocation.LEFT;
            else if (Math.max(distL, distR) >= Math.min(distL, distR) * 1.5)
                return Math.max(distL, distR) == distL ? ToboSigma.SkystoneLocation.CENTER : ToboSigma.SkystoneLocation.RIGHT;
        } else // red side
        {
            if (Math.abs(distL - distR) <= 10)
                return ToboSigma.SkystoneLocation.RIGHT;
            else if (Math.max(distL, distR) >= Math.min(distL, distR) * 1.5)
                return Math.max(distL, distR) == distL ? ToboSigma.SkystoneLocation.LEFT : ToboSigma.SkystoneLocation.CENTER;
        }
        return ToboSigma.SkystoneLocation.UNKNOWN;
    }

    public boolean stoneCollected() {
        if (Thread.interrupted()) return false;
        if (FRDistance == null) {
            return false;
        }
        double dist = getDistanceColor(FRDistance);
        if (dist > 50)
            return false;
        else
            return true;
    }
}
