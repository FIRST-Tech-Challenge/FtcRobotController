package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.List;

import static com.qualcomm.hardware.lynx.commands.core.LynxInjectDataLogHintCommand.charset;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
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
 */
public class MechChassis extends Logger<MechChassis> implements Configurable {

    final private CoreSystem core;

    public enum TeleOpDriveMode { // for TeleOp
        STOP,      // not moving
        STRAIGHT,  // driving in a straight line utilizing orientation sensor to correct itself,
        //  all servos are set to the same position
        ROTATE,    // rotating in place, all servos are set on tangent lines to a circle drawn
        //  through the centers of 4 wheels.
        STEER      // motor power and servos in the direction of movement are controlled by
        //  the driver; opposite servos are in central position
    }

    public enum AutoDriveMode {
        STOP,                      // stop at the end of each moving routine with correction
        STOP_NO_CORRECTION,        // stop at the end of each moving routine without correction
        CONTINUE,                  // do not stop at end end of each moving routine but do correction
        CONTINUE_NO_CORRECTION     // do not stop at end end of each moving routine without correction
    }

    public enum ShootingTarget {
        TOWER,      // Tower Goal
        PSHOT_L,    // Power Shot Left
        PSHOT_M,    // Power Shot Middle
        PSHOT_R     // Power Shot Right
    }

    public static class Point { // point for the robot position (x, y, h)
        public double x,y,h;
        public Point(double lx, double ly, double lh) {
            x=lx; y=ly; h=lh;
        }
    }

    void dumpEvent(String s) throws IOException {
        byte data[] = s.getBytes();
        simOS.write(data);
        simEvents += s;
    }

    // the following 4 ratio values are used to normalize 4 wheel motors to the same speed
    // whenever changing a wheel motor, it must be calibrated again
    /* for GoBilda 435 motor set:
    private double ratioFL = 1.0;
    private double ratioFR = 0.8847;
    private double ratioBL = 0.9348;
    private double ratioBR = 0.9315;
    */
    /* for GoBilda 1125 motor set: */
    private double ratioFL = 14460.0/14503.0;
    private double ratioFR = 14460.0/14710.0;
    private double ratioBL = 14460.0/14756.0;
    private double ratioBR = 1.0;

    private double left_ratio = 1.0; // slow down ratio for left wheels to go straight
    private double right_ratio = 1.0; // slow down ratio for right wheels to go straight
    private double front_ratio = 1.0; // slow down ratio for front wheels to go 90 degree
    private double back_ratio = 0.975; // slow down ratio for front wheels to go 90 degree

    private double fixedStopDist = 18; // stop distance for 152.4 cm /sec


    // distance between the centers of left and right wheels, inches
    private double track = 15.75;
    // distance between the centers of front and back wheels, inches
    private double wheelBase = 13;
    // wheel radius, inches
    private double wheelRadius = 2.0;
    // minimum power that should be applied to the wheel motors for robot to start moving
    private double minPower = 0.1;
    // maximum power that should be applied to the wheel motors
    private double maxPower = 0.999;
    private double slowDownSpeed = 0.25;
    private double minPowerHorizontal = 0.3;
    private double initX = 0;
    private double initY = 0;

    private double maxRange = 127; // max range sensor detectable
    private double defaultScale = 1.0;
    private double mecanumForwardRatio = 0.8;
    public double chassisAligmentPower = 0.15;
    public double chassisAligmentPowerMin = 0.13;
    private double init_x_cm = 0.0;
    private double init_y_cm = 0.0;
    private double init_heading = 0;

    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;

    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    public CombinedOrientationSensor orientationSensor;
    public double auto_target_x = 0;
    public double auto_target_y = 0;
    public double auto_dist_err = 0;
    public double auto_degree_err = 0;
    public double auto_power = 0;
    public double auto_loop_time = 0;
    public double auto_travel_p = 0;
    public double auto_max_speed = 0;
    public double auto_max_calc_speed = 0;
    public double auto_exit_speed = 0;
    public double auto_stop_early_dist = 0;

    private TeleOpDriveMode teleOpDriveMode = TeleOpDriveMode.STOP;      // current drive mode
    private AutoDriveMode autoDriveMode = AutoDriveMode.STOP;
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation
    private double curHeading = 0;
    private boolean useScalePower = true;//
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor
    private boolean useOdometry = true;
    private boolean normalizeMode = true;
    private boolean showEncoderDetail = false; // enable the chassis encoders

    private String simEvents="";
    public FileOutputStream simOS;
    private boolean simulation_mode = false;

    public void set_simulation_mode(boolean val) {
        simulation_mode = val;
    }
    public String getSimEvents() { return simEvents; }

    public void toggleNormalizeMode(){
        normalizeMode = !normalizeMode;
    }
    public boolean getNormalizeMode(){
        return normalizeMode;
    }

    public void switchAutoMode() {
        switch (autoDriveMode) {
            case STOP: setAutoDriveMode(AutoDriveMode.STOP_NO_CORRECTION); break;
            case STOP_NO_CORRECTION: setAutoDriveMode(AutoDriveMode.CONTINUE); break;
            case CONTINUE: setAutoDriveMode(AutoDriveMode.CONTINUE_NO_CORRECTION); break;
            case CONTINUE_NO_CORRECTION: setAutoDriveMode(AutoDriveMode.STOP); break;
        }
    }

    public AutoDriveMode getAutoDriveMode() { return autoDriveMode;}
    public void setAutoDriveMode(AutoDriveMode mode) { autoDriveMode = mode;}

    public void configure_IMUs(Configuration configuration, boolean noReset) {
        orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
        orientationSensor.configure(configuration.getHardwareMap(), noReset, "imu", "imu2");
    }

    public void reset_imus() {
        orientationSensor.reset();
    }

    List<LynxModule> allHubs;

    //odometry motors
    DcMotorEx verticalLeftEncoder;
    DcMotorEx verticalRightEncoder;
    DcMotorEx horizontalEncoder;
    OdometryGlobalCoordinatePosition GPS;
    final double ODO_COUNTS_PER_INCH = 303.7;
    final double ODO_COUNTS_PER_CM = ODO_COUNTS_PER_INCH / 2.54;

    String rfName = "motorFR" , lfName = "motorFL";
    String rbName = "motorBR";
    String lbName = "motorBL";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    final double WHEEL_CM_PER_ROTATION = Math.PI*3.8;       // ~11.938cm, odometry wheel is 38 mm
    final double TICKS_PER_CM = 1446/WHEEL_CM_PER_ROTATION; // ~30.1557, number of encoder ticks per cm of driving

    final double ODO_ENC_RATIO = 1446.0/(291.2*2);  // ratio to ticks-per-rotation with goBilda 5202 1150rpm is 145.6 x 2 gear up
                                                // Odometry encoder E8T-360 is 360 ticks-per-rotation

    public void setGlobalPosUpdate(OdometryGlobalCoordinatePosition val) { GPS =val;}
    public OdometryGlobalCoordinatePosition getGPS() { return GPS; }
    public double odo_count_per_inch() {return ODO_COUNTS_PER_INCH;}
    public double odo_count_per_cm() {return ODO_COUNTS_PER_CM;}
    public DcMotorEx verticalLeftEncoder(){ return verticalLeftEncoder; }
    public DcMotorEx verticalRightEncoder(){ return verticalRightEncoder; }
    public DcMotorEx horizontalEncoder(){ return horizontalEncoder; }

    public double getMecanumForwardRatio() {
        return mecanumForwardRatio;
    }

    public void configureOdometry(Telemetry telemetry) {
        if (!useOdometry) return;
        GPS = new OdometryGlobalCoordinatePosition(verticalLeftEncoder(), verticalRightEncoder(), horizontalEncoder(), odo_count_per_inch(), 75);
        GPS.set_orientationSensor(orientationSensor);
        // GPS.reverseRightEncoder();
        // GPS.reverseLeftEncoder();
        GPS.reverseNormalEncoder();
        GPS.set_init_pos(init_x_cm*odo_count_per_cm(), init_y_cm*odo_count_per_cm(), init_heading);
        setupGPSTelemetry(telemetry);
    }

    public void updateConfigureForGG() {
        GPS.reverseRightEncoder();
        GPS.reverseLeftEncoder();
    }

    public void set_pos_for_simulation(double x, double y, double heading) {
        init_x_cm = x;
        init_y_cm = y;
        init_heading = heading;
    }

    public void set_init_pos(double x, double y, double heading) {
        if (simulation_mode) {
            try {
                dumpEvent(String.format("set_init_pos: %3.0f, %3.0f, %3.0f\n", x, y, targetHeading));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        init_x_cm = x;
        init_y_cm = y;
        init_heading = heading;
        if (GPS!=null) {
            GPS.set_init_pos(x,y,heading);
        }
        auto_target_y = init_y_cm;
        auto_target_x = init_x_cm;
        targetHeading = init_heading;
    }

    public double getInit_x_cm() {
        return init_x_cm;
    }

    public double getInit_y_cm() {
        return init_y_cm;
    }

    public double getInit_heading() {
        return init_heading;
    }

    public double odo_x_pos_inches() {
        if (GPS ==null) return init_x_cm/2.54;
        return GPS.returnXCoordinate()/odo_count_per_inch();
    }

    public double odo_x_pos_cm() {
        if (GPS ==null) return init_x_cm;
        return GPS.returnXCoordinate()/odo_count_per_cm();
    }

    public double odo_y_pos_inches() {
        if (GPS ==null) return init_y_cm/2.54;
        return GPS.returnYCoordinate()/odo_count_per_inch();
    }

    public double odo_y_pos_cm() {
        if (GPS ==null) return init_y_cm;
        return GPS.returnYCoordinate()/odo_count_per_cm();
    }

    public double odo_x_speed_cm() { // horizontal speed as cm/sec
        if (GPS ==null) return 0;
        return GPS.getXSpeedDegree() / 360.0 * WHEEL_CM_PER_ROTATION * ODO_ENC_RATIO;
    }

    public double odo_y_speed_cm() { // forward speed as cm/sec
        if (GPS ==null) return 0;
        return GPS.getYSpeedDegree() / 360.0 * WHEEL_CM_PER_ROTATION * ODO_ENC_RATIO;
    }

    public double odo_speed_cm() {
       double speed = 0;
       speed = Math.hypot(odo_x_speed_cm(), odo_y_speed_cm());
       return speed;
    }

    public double odo_heading() { // aways turn [-180..180]
        if (GPS ==null) return 0;
        double heading = (GPS.returnOrientation());
        if (heading>180) heading -= 360;
        else if (heading<-180) heading += 360;
        return heading;
    }

    public double getLeft_ratio() { return left_ratio; }
    public double getRight_ratio() { return left_ratio; }
    public double getFront_ratio() { return front_ratio; }
    public double getBack_ratio() { return back_ratio; }

    public double powerScale() { return defaultScale; }
    public double getDefaultScale() {
        return defaultScale;
    }
    public void setDefaultScale(double val) {
        defaultScale = val;
    }

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public void enableImuTelemetry(Configuration configuration) {
        setImuTelemetry = true;
        if (orientationSensor==null) {
            configure_IMUs(configuration, false);
        }

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

    @Adjustable(min = 0, max = 360.0, step = 1)
    public double getInitX() { return initX; }
    public void setInitX(double x) {
        this.initX = x;
    }

    @Adjustable(min = 0, max = 360.0, step = 1)
    public double getInitY() { return initY; }
    public void setInitY(double y) {
        this.initY = y;
    }

    @Override
    public String getUniqueName() {
        return "chassis";
    }

    /**
     * SwerveChassis constructor
     */
    public MechChassis(CoreSystem core) {
        this.core = core;
    }

    /**
     * Used only for ToboRuckus, old code
     */
    @Deprecated
    public MechChassis() {
        this.core = new CoreSystem(); // Prevents null pointer exception
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration, boolean auto, boolean noReset) {
        // set up motors / sensors as wheel assemblies
        if (simulation_mode) return;

        motorFL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lfName);
        motorFR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rfName);
        motorBL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lbName);
        motorBR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rbName);

        if (motorFL==null)
            return;

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.getMotorType();
        // map odometry encoders
        verticalLeftEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalLeftEncoderName);
        verticalRightEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalRightEncoderName);
        horizontalEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, horizontalEncoderName);

        verticalLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //verticalLeftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //verticalRightEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable bulk read mode to speed up the encoder reads
        allHubs = configuration.getHardwareMap().getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (auto || setImuTelemetry) {
            configure_IMUs(configuration, noReset);
        }

        // register chassis as configurable component
        configuration.register(this);
    }


    public void driveStraight(double power, double cm, double degree, long timeout_sec) throws InterruptedException {
        driveStraight(power, cm, degree, 0.8, timeout_sec);
    }

    public void driveStraight(double power, double cm, double degree, double slowDownPercent, long timeout_sec) throws InterruptedException {
        // to-do: Sasha to compute x_dist/y_dist based on current heading
        double x_dist = cm * Math.sin(Math.toRadians(degree)) * Math.signum(power);
        double y_dist = cm * Math.cos(Math.toRadians(degree)) * Math.signum(power);
        double target_x = x_dist + odo_x_pos_cm();
        double target_y = y_dist + odo_y_pos_cm();
        auto_target_x = target_x;
        auto_target_y = target_y;

        power = power * Math.signum(cm);
        double fixed_heading = odo_heading();

        driveTo(power, target_x, target_y, fixed_heading, false, timeout_sec);
    }


    public void driveThrough(double power, Point[] points, boolean useRotateTo,double timeout_sec) throws InterruptedException {
        int nPoints = points.length;
        if (nPoints<1) return;
        setAutoDriveMode(AutoDriveMode.CONTINUE_NO_CORRECTION);
        for (int i=0; i<nPoints-1; i++) {
            driveTo(power, points[i].x, points[i].y, points[i].h, false, timeout_sec);
            if (Thread.interrupted()) break;
        }
        setAutoDriveMode(AutoDriveMode.STOP);
        driveTo(power, points[nPoints-1].x, points[nPoints-1].y, points[nPoints-1].h, true , timeout_sec);
    }

    public void driveTo(double power, double target_x, double target_y, double timeout_sec) throws InterruptedException {
        double cur_x = odo_x_pos_cm();
        double cur_y = odo_y_pos_cm();
        double desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
        driveTo(power, target_x, target_y, desiredDegree, true, timeout_sec);
        // auto_degree_err = desiredDegree;
    }

    public void driveTo(double power, double target_x, double target_y, double target_heading, boolean useRotateTo, double timeout_sec) throws InterruptedException {
        if (simulation_mode) { // simulation mode
            try {
                dumpEvent (String.format("driveTo: %3.0f, %3.0f, %3.0f\n", target_x, target_y, target_heading));
            } catch (IOException e) {
                e.printStackTrace();
            }
            set_pos_for_simulation(target_x,target_y,target_heading);
            return;
        }
        long iniTime = System.currentTimeMillis();
        double current_heading = odo_heading();
        double cur_x = odo_x_pos_cm(), prev_x = cur_x, init_x=cur_x;
        double cur_y = odo_y_pos_cm(), prev_y = cur_y, init_y=cur_y;
        double desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
        double currentAbsDiff = abs(desiredDegree - current_heading) > 180 ? 360 - abs(desiredDegree - current_heading) : abs(desiredDegree - current_heading);
        double finalAbsDiff =  abs(target_heading - current_heading) > 180 ? 360 - abs(target_heading - current_heading) : abs(target_heading - current_heading);
        boolean rotateFirst = true;
        double stage_one_heading=current_heading;
        double slowDownPercent = 0.8;

        // if desirableDegree is between current_heading and target_heading, rotate to desirableDegress first
        // else if target_heading is between current_heading and desirableDegree, rotate to target_heading first
        if (current_heading>desiredDegree+20 && desiredDegree>=target_heading) {
            stage_one_heading = desiredDegree + 20;
        } else if (current_heading<desiredDegree-20 && desiredDegree<=target_heading) {
            stage_one_heading = desiredDegree - 20;
        } else if (current_heading>target_heading+20 && target_heading>=desiredDegree) {
            // rotate to about target_heading
            stage_one_heading = target_heading + 20;
        } else if (current_heading<target_heading-20 && target_heading<=desiredDegree) {
            // rotate to about target_heading
            stage_one_heading = target_heading - 20;
        } else {
            rotateFirst = false;
        }

        if (useRotateTo && rotateFirst && (stage_one_heading!=current_heading)) {
            rawRotateTo(power, stage_one_heading, true, timeout_sec);
            cur_x = odo_x_pos_cm();
            cur_y = odo_y_pos_cm();
            desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
        }

        double error_cm = 2.0;  // to-do: error_cm should depend on degree
        double powerUsed = (Math.abs(power)<minPower?minPower*Math.signum(power):power);
        double x_dist = target_x - cur_x;
        double y_dist = target_y - cur_y;
        double total_dist = Math.hypot(x_dist,y_dist);
        double traveledPercent = 0;
        double save_target_heading = target_heading;

        // Temporarily change target_heading to go straight
        if (useRotateTo) {
            target_heading = odo_heading();
        }
        double[] motorPowers = {0, 0, 0, 0};
        // slow down stuff
        double[] s = getSlowDownParameters(target_heading, getCurHeading(), power);
        slowDownPercent = s[0];
        double decreasePercent = s[1];
        double minPowerForAngle = s[2];

        double init_loop_time = System.currentTimeMillis();
        int loop_count = 0;
        double cur_s;
        double expectedStopDist;
        double remDistance;
        auto_max_calc_speed = 0; prev_x=cur_x; prev_y=cur_y;
        double prev_time = init_loop_time;
        double cur_time = prev_time;
        auto_max_speed = auto_exit_speed = auto_stop_early_dist = 0;
        while((traveledPercent<.99) && (System.currentTimeMillis() - iniTime < timeout_sec * 1000)) {
            traveledPercent = Math.hypot(cur_y - init_y, cur_x-init_x)/total_dist;
            auto_travel_p = traveledPercent;
            if (traveledPercent>0.99) {
                auto_exit_speed = odo_speed_cm(); // exiting speed
                auto_stop_early_dist = 0.01*total_dist;
                break;
            }
            cur_s = odo_speed_cm();
            auto_max_speed = Math.max(cur_s, auto_max_speed);
            //auto_max_xspeed = Math.max(odo_x_speed_cm(), auto_max_xspeed);
            //auto_max_yspeed = Math.max(odo_y_speed_cm(), auto_max_yspeed);
            if (autoDriveMode!= AutoDriveMode.CONTINUE_NO_CORRECTION) {
                if (traveledPercent > slowDownPercent && cur_s > 30 && powerUsed > slowDownSpeed) {
                    powerUsed = 0.4;
                }
            }
            if (traveledPercent<0.9) {
                desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
            }
            if (Thread.interrupted()) break;
            //move
            motorPowers = angleMove(desiredDegree, powerUsed, true,
                    (autoDriveMode== AutoDriveMode.CONTINUE_NO_CORRECTION?desiredDegree:target_heading));

            remDistance = Math.hypot(target_x- cur_x, target_y - cur_y);
            { // over-shoot prevention
                expectedStopDist = Math.pow(cur_s / 152.4, 2) * fixedStopDist;
                if ((remDistance - expectedStopDist) < .001) {
                    // we need to measure fixedStopDist ( overshoot for any speed, then we need to change 20. to that speed
                    auto_exit_speed = odo_speed_cm(); // exiting speed
                    auto_stop_early_dist = expectedStopDist;
                    break;
                }
            }
            cur_x = odo_x_pos_cm();
            cur_y = odo_y_pos_cm();
            loop_count ++;
            cur_time = System.currentTimeMillis();
            if ((cur_time-prev_time)>=100) { // calculate speed every 100 ms
                double speed = Math.hypot(cur_x-prev_x, cur_y-prev_y)/(cur_time-prev_time)*1000.0;
                auto_max_calc_speed = Math.max(speed, auto_max_calc_speed);
                prev_time=cur_time; prev_x=cur_x; prev_y=cur_y;
            }
            // info("Odo-y-speed = %3.2f", verticalRightEncoder.getVelocity(AngleUnit.DEGREES));
        }
        double end_loop_time = System.currentTimeMillis();
        if (autoDriveMode== AutoDriveMode.STOP_NO_CORRECTION || autoDriveMode== AutoDriveMode.STOP) {
            stopNeg(motorPowers);
        }
        if (loop_count>0)
            auto_loop_time = (end_loop_time-init_loop_time)/(double)loop_count;

        target_heading = save_target_heading;

        current_heading = odo_heading();
        currentAbsDiff = abs(target_heading - current_heading) > 180 ? 360 - abs(target_heading - current_heading) : abs(target_heading - current_heading);
        if (useRotateTo||(autoDriveMode== AutoDriveMode.STOP) && (currentAbsDiff > 1.2) && !Thread.interrupted()) {
            rotateTo(Math.abs(power), target_heading, timeout_sec);
        }
        if (autoDriveMode== AutoDriveMode.STOP) {
            // The following code is for error estimation, should be commented out in competition
            sleep(200);
            auto_dist_err = Math.hypot(odo_x_pos_cm() - target_x, odo_y_pos_cm() - target_y);
            auto_degree_err = Math.abs(target_heading - odo_heading());
            //tl.addData("speed: ", odo_speed_cm());
            //tl.update();
        }
    }

    public double getSlowDownPower(double traveledPercent, double slowDownPercent, double decreaseP, double power, double minPowerForAngle){
        double apower = Math.abs(power);
        double pow;
        double percentPow;
        if (traveledPercent < .25 + .75 * slowDownPercent)
            //pow = .25 * minPower + .75 * apower;
            percentPow = decreaseP;
        else if (traveledPercent < .5 + .5 * slowDownPercent)
            //pow = .5 * minPower + .5 * apower;
            percentPow = decreaseP * .6;
        else if (traveledPercent < .75 + .25 * slowDownPercent) {
            //pow = .75 * minPower + .25 * apower;
            percentPow = decreaseP * .3;
        } else {
            //pow = minPower;
            percentPow = 0;
        }
        pow = percentPow * apower + (1-percentPow) * minPowerForAngle;
        if (pow < minPowerForAngle) pow = minPowerForAngle;
        return  pow * Math.signum(power);
    }

    public double[] getSlowDownParameters(double directionAngle, double heading, double power){
        double movementAngle = Math.abs(directionAngle - heading);
        movementAngle = Math.min(180-movementAngle, movementAngle);// movementaAngle but from 0 to 90
        movementAngle = 1 - movementAngle / 90.;// movemebt angle from 0 (horizontal, slowest) to 1(vertical, fastest
        double minPowerForAngle = minPowerHorizontal - movementAngle * (minPowerHorizontal - minPower);
        double powWeight = 1, movementAngleWeight = 1;// these numebrs need to be tested and maybe changed
        double fast = (powWeight * power + movementAngleWeight * movementAngle) / (powWeight + movementAngleWeight);  // how fast the robot is going to go from 0 to 1
        double fastSlowDownP = .7, slowSlowDownP = .85;
        double fastDecreaseP = .75, slowDecreaseP = .4;
        double slowDownP = fastSlowDownP + (1-fast) * (slowSlowDownP - fastSlowDownP);// slow down percent - slow down sooner when going faster
        double decreaseP = fastDecreaseP + (1-fast) * (slowDecreaseP - fastDecreaseP); // how much we decrease power in the first step - slower has steeper decrease

        return new double[] {slowDownP, decreaseP, minPowerForAngle};
    }
    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        if (simulation_mode) return;
        if (Math.abs(power)<minPower) power=minPower*Math.signum(power);
        motorFL.setPower(sgn * power * left_ratio * ratioFL);
        motorFR.setPower(sgn * power * right_ratio * ratioFR);
        motorBL.setPower(sgn * power * left_ratio * ratioBL);
        motorBR.setPower(sgn * power * right_ratio * ratioBR);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        if (simulation_mode) return;
        if (Math.abs(power)<1.75*minPower) power=1.5*minPower*Math.signum(power);
        motorFL.setPower(sgn * power * front_ratio * ratioFL);
        motorFR.setPower(-sgn * power * front_ratio * ratioFR);
        motorBL.setPower(-sgn * power * back_ratio * ratioBL);
        motorBR.setPower(sgn * power * back_ratio * ratioBR);
    }
    public double[] angleMove(double directionAngle, double power, boolean headingCorrection, double fixed_heading){

        // auto_travel_p = directionAngle;

        double cur_heading = odo_heading();
        double degree_diff = Math.abs(cur_heading-fixed_heading);

        // to-do: need to handle gap from 179 to -179
        double cur_left_to_right_ratio = 1.0;
        boolean slow_down_left=false, slow_down_right=false;
        if (headingCorrection && (degree_diff>1.0) && power>slowDownSpeed) { // for Y axle correction
            slow_down_left = ((cur_heading-fixed_heading>0) && directionAngle>-65 && directionAngle<65) ||
                    (((cur_heading-fixed_heading<0) && directionAngle>115 && directionAngle<-115));
            slow_down_right = ((cur_heading-fixed_heading<0) && directionAngle>-65 && directionAngle<65) ||
                    (((cur_heading-fixed_heading>0) && directionAngle>115 && directionAngle<-115));
            if (slow_down_left||slow_down_right) {
                cur_left_to_right_ratio = (1.0 - degree_diff * 0.05);
                if (cur_left_to_right_ratio<0.7) cur_left_to_right_ratio=0.7;
            }
        }
        // auto_dist_err = degree_diff;
        double cur_front_to_back_ratio = 1.0;
        boolean slow_down_front=false, slow_down_back=false;
        if (headingCorrection && (degree_diff>1.0) && (cur_left_to_right_ratio>0.9) && power>slowDownSpeed) {
            // for Y axle correction
            slow_down_front = ((cur_heading-fixed_heading>0) && directionAngle>=45 && directionAngle<=135) ||
                    (((cur_heading-fixed_heading<0) && directionAngle>=-135 && directionAngle<=-45));
            slow_down_back = ((cur_heading-fixed_heading<0) && directionAngle>=45 && directionAngle<=135) ||
                    (((cur_heading-fixed_heading>0) && directionAngle>=-135 && directionAngle<=-45));
            if (slow_down_front||slow_down_back) {
                cur_front_to_back_ratio = (1.0 - degree_diff * 0.05);
                if (cur_front_to_back_ratio<0.7) cur_front_to_back_ratio=0.7;
            }
        }

        double[] motorPowers  = new double[4];
        motorPowers[0] = (Math.sin(Math.toRadians(directionAngle - fixed_heading))+ Math.cos(Math.toRadians(directionAngle- fixed_heading)));
        motorPowers[1] = (Math.cos(Math.toRadians(directionAngle- fixed_heading))- Math.sin(Math.toRadians(directionAngle - fixed_heading)));
        motorPowers[2] = motorPowers[1];
        motorPowers[3] = motorPowers[0];
        double max = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        if (slow_down_left) { // slow-down left
            motorPowers[0] *= cur_left_to_right_ratio;
            motorPowers[2] *= cur_left_to_right_ratio;
        } else if (slow_down_right) { // slow down right
            motorPowers[1] *= cur_left_to_right_ratio;
            motorPowers[3] *= cur_left_to_right_ratio;
        }
        // auto_travel_p=cur_left_to_right_ratio;
        if (slow_down_front) { // make front motors slower
            motorPowers[0] = motorPowers[0] * Math.abs(power) * ratioFL * cur_front_to_back_ratio / max;
            motorPowers[1] = motorPowers[1] * Math.abs(power) * ratioFR * cur_front_to_back_ratio / max;
            motorPowers[2] = motorPowers[2] * Math.abs(power) * ratioBL / max;
            motorPowers[3] = motorPowers[3] * Math.abs(power) * ratioBR / max;
            motorFL.setPower(motorPowers[0]);
            motorFR.setPower(motorPowers[1]);
            motorBL.setPower(motorPowers[2]);
            motorBR.setPower(motorPowers[3]);
        } else { // slow down back (or keep same)
            motorPowers[0] = motorPowers[0] * Math.abs(power) * ratioFL / max;
            motorPowers[1] = motorPowers[1] * Math.abs(power) * ratioFR / max;
            motorPowers[2] = motorPowers[2] * Math.abs(power) * ratioBL * cur_front_to_back_ratio / max;
            motorPowers[3] = motorPowers[3] * Math.abs(power) * ratioBR * cur_front_to_back_ratio / max;

            motorFL.setPower(motorPowers[0]);
            motorFR.setPower(motorPowers[1]);
            motorBL.setPower(motorPowers[2]);
            motorBR.setPower(motorPowers[3]);
        }
        return motorPowers;
    }

    public void freeStyle(double fl, double fr, double bl, double br, boolean normalized) {
        if (simulation_mode) return;
        if (normalized) {
            motorFL.setPower(fl * ratioFL);
            motorFR.setPower(fr * ratioFR);
            motorBL.setPower(bl * ratioBL);
            motorBR.setPower(br * ratioBR);
        } else {
            motorFL.setPower(fl);
            motorFR.setPower(fr);
            motorBL.setPower(bl);
            motorBR.setPower(br);
        }
    }

    public void forward(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0 forward
        //        < 0 backward
        if (simulation_mode) return;

        power = Math.abs(power);
        boolean count_up = (Math.signum(inches)>0);
        double error_inches = 0.1;
        double target_y = inches+odo_y_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_y = odo_y_pos_inches(), prev_y=cur_y;
        while ((Math.abs(cur_y-target_y) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            yMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_y>=target_y-error_inches) break;
            } else {
                if (cur_y<=target_y+error_inches) break;
            }
                prev_y=cur_y;
            cur_y = odo_y_pos_inches();
        }
        stop();
    }

    public void crab(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0, crab right 90 degree
        //        < 0, crab left 90 degree
        if (simulation_mode) return;
        power = Math.abs(power);
        if (Math.abs(power)<minPower) power=minPower*Math.signum(power);
        boolean count_up = (Math.signum(inches)>0);

        double error_inches = 0.1;
        double target_x = inches+odo_x_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_x = odo_x_pos_inches(), prev_x=cur_x;
        while ((Math.abs(cur_x-target_x) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            xMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_x>=target_x-error_inches) break;
            } else {
                if (cur_x<=target_x+error_inches) break;
            }
            prev_x=cur_x;
            cur_x = odo_x_pos_inches();
        }
        stop();
    }

    /**
     * pivot and turn
     *
     * @param sgn   can be either +1 or -1(+1 for clockwise, -1 for counter clockwise)
     * @param power must be in range [0,1]
     */
    public void turn(int sgn, double power) {
        if (simulation_mode) return;
        if (Math.abs(power)<minPower) power=minPower*Math.signum(power);
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(-sgn * power);
    }

    public void chassis_test() throws InterruptedException {
        if (simulation_mode) return;

        motorFR.setPower(1*ratioFR);
        motorFL.setPower(1*ratioFL);
        motorBR.setPower(1*ratioBR);
        motorBL.setPower(1*ratioBL);
        sleep(5000);
        stop();

//        motorFR.setPower(0.1);
//        sleep(1000);
//        motorFR.setPower(0);
//
//        motorBR.setPower(0.1);
//        sleep(1000);
//        motorBR.setPower(0);
//
//        motorFL.setPower(0.1);
//        sleep(1000);
//        motorFL.setPower(0);
//
//        motorBL.setPower(0.1);
//        sleep(1000);
//        motorBL.setPower(0);
    }

    /**
     * turning while driving
     *
     * @param power         must be in range [0,1]
     * @param turningFactor int range [-1,+1] (+1 for turning right, -1 for turning left)
     */
    public void carDrive(double power, double turningFactor) {
        if (simulation_mode) return;
        if (Math.abs(power)<minPower) power=minPower*Math.signum(power);
        if (turningFactor > 0) {
            turningFactor = 1 - turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            } else {
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            }
        } else {
            turningFactor = 1 + turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            } else {
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            }
        }

    }

    public void setRunMode(DcMotor.RunMode rm) {
        if (simulation_mode) return;
        motorFL.setMode(rm);
        motorFR.setMode(rm);
        motorBL.setMode(rm);
        motorBR.setMode(rm);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        if (simulation_mode) return;
        motorFL.setZeroPowerBehavior(zpb);
        motorFR.setZeroPowerBehavior(zpb);
        motorBL.setZeroPowerBehavior(zpb);
        motorBR.setZeroPowerBehavior(zpb);
    }

    public void stop() {
        if (simulation_mode) return;
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void stopNeg(double[] motorPowers) throws InterruptedException {
        if (simulation_mode) return;
        double speed = odo_speed_cm();
        motorFL.setPower(-Math.signum(motorPowers[0] * .01));
        motorFR.setPower(-Math.signum(motorPowers[1] * .01));
        motorBL.setPower(-Math.signum(motorPowers[2] * .01));
        motorBR.setPower(-Math.signum(motorPowers[3] * .01));
        sleep((int)(100*speed/150));
        stop();
    }

    public void reset() {
        if (simulation_mode) return;
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        teleOpDriveMode = TeleOpDriveMode.STOP;
        targetHeading = 0;
    }


    public enum Direction {
        FRONT, LEFT, RIGHT, BACK;
    }

    public enum Wall {
        LEFT, RIGHT;
    }

    public double getCurHeading() {
        return curHeading;
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Pwr/Scale/Mode", new Func<String>() {
            @Override
            public String value() {
                return String.format("%.2f / %.1f / %s", motorFL.getPower(), getDefaultScale(),
                        (simulation_mode?"Simulation":(getNormalizeMode()?"Normalized":"Speedy")));
            }
        });

        if (showEncoderDetail) {
            if (motorFL != null) {
                line.addData("FL", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return motorFL.getCurrentPosition();
                    }
                });
            }
            if (motorFR != null) {
                line.addData("FR", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return motorFR.getCurrentPosition();
                    }
                });
            }
            if (motorBL != null) {
                line.addData("BL", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return motorBL.getCurrentPosition();
                    }
                });
            }
            if (motorBR != null) {
                line.addData("BR", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return motorBR.getCurrentPosition();
                    }
                });
            }

            if (horizontalEncoder != null) {
                line.addData("row X", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return horizontalEncoder.getCurrentPosition();
                    }
                });
            }
            if (verticalLeftEncoder != null) {
                line.addData("row Y-Left", "%d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return verticalLeftEncoder.getCurrentPosition();
                    }
                });
            }
            if (verticalRightEncoder != null) {
                line.addData("row Y-Right", "%d\n", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return verticalRightEncoder.getCurrentPosition();
                    }
                });
            }
        }

        // setupGPSTelemetry(telemetry);
        setupIMUTelemetry(telemetry);


//        telemetry.addLine().addData("M", new Func<String>() {
//            @Override
//            public String value() {
//                return teleOpDriveMode.name();
//            }
//        }).addData("Head", new Func<String>() {
//            @Override
//            public String value() {
//                if (teleOpDriveMode != TeleOpDriveMode.STRAIGHT) return "N/A";
//                return String.format("%+.1f (%+.2f)", targetHeading, headingDeviation);
//            }
//        }).addData("Adj", new Func<String>() {
//            @Override
//            public String value() {
//                if (teleOpDriveMode != TeleOpDriveMode.STRAIGHT) return "N/A";
//                return String.format("%+.1f", servoCorrection);
//            }
//        });
    }

    public void setupIMUTelemetry(Telemetry telemetry) {
        if ((orientationSensor!=null) && setImuTelemetry) {
            Telemetry.Line line = telemetry.addLine();
            line.addData("imuC", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

    }

    public void setupGPSTelemetry(Telemetry telemetry) {
        if (GPS !=null) {
            Telemetry.Line line = telemetry.addLine();

//            line.addData("driveTo (stop-sp,stop-cm,max-dod-sp,max-calc-sp)", new Func<String>() {
//                @Override
//                public String value() {
//                    return String.format("(%5.1f,%5.1f,%5.1f,%5.1f)\n", auto_exit_speed, auto_stop_early_dist, auto_max_speed, auto_max_calc_speed);
//                }
//            });
            line.addData("Odo-pos (x,y,angle)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%2.0f, %2.0f, %2.2f)", odo_x_pos_cm(), odo_y_pos_cm(), odo_heading());
                }
            });
            line.addData("Raw (x,ly,ry)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%2.0f,%2.0f,%2.0f)", GPS.XEncoder(),
                            GPS.leftYEncoder(),
                            GPS.rightYEncoder());
                }
            });
        }
    }

    public void setupEncoders(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (motorFL != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFL.getCurrentPosition();
                }
            });
        }
        if (motorFR != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFR.getCurrentPosition();
                }
            });
        }
        if (motorBL != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBL.getCurrentPosition();
                }
            });
        }
        if (motorBR != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBR.getCurrentPosition();
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

    public void rawRotateTo(double power, double finalHeading, boolean stopEarly, double timeout_sec) throws InterruptedException {
        if (Thread.interrupted()) return;
        if (simulation_mode) { // simulation mode
            set_pos_for_simulation(odo_x_pos_cm(),odo_y_pos_cm(),finalHeading);
            try {
                dumpEvent (String.format("rawRotateTo: %3.0f %3.0f %3.0f\n", odo_x_pos_cm(), odo_y_pos_cm(), finalHeading));
            } catch (IOException e) {
                e.printStackTrace();
            }
            return;
        }

        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = odo_heading();
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
        //for (WheelAssembly wheel : wheels)
          //  wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        //driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        turn((int)Math.signum(deltaD), power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = odo_heading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            if (Thread.interrupted()) return;
            double currentHeading = odo_heading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? power * 10.0 : 0.6)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;

            if (System.currentTimeMillis() - iniTime > timeout_sec*1000) break;
            //stop pressed, break
            if (Thread.interrupted()) return;
            lastReading = currentHeading;
//            sleep(0);
            // yield handler
            //TaskManager.processTasks();
            this.core.yield();
        }
        if (Thread.interrupted()) return;
        if (autoDriveMode== AutoDriveMode.STOP_NO_CORRECTION || autoDriveMode== AutoDriveMode.STOP) {
            stop();
        }

        useScalePower = true;
    }

    public void rotateDegrees(double power, double degree) throws InterruptedException {
        double iniHeading = odo_heading();
        double finalHeading = iniHeading + degree;
        if (finalHeading>180) finalHeading-=360;       //  190 become -170
        else if (finalHeading<-180) finalHeading+=360; // -190 become  170
        rotateTo(power, finalHeading, 3000);
    }

    static final double degreeToRad = PI / 180;
    static final double radToDegree = 180 / PI;

    public void rotateTo(double power, double finalHeading) throws InterruptedException {
        rotateTo(power, finalHeading, 4000);
    }

    public void rotateTo(double power, double finalHeading, double timeout_sec) throws InterruptedException {
        rotateTo(power, finalHeading, timeout_sec, true,true);
    }

    public void rotateTo(double power, double finalHeading, double timeout_sec, boolean changePower, boolean finalCorrection) throws InterruptedException {
        if (simulation_mode) { // simulation mode
            set_pos_for_simulation(odo_x_pos_cm(),odo_y_pos_cm(),finalHeading);
            try {
                dumpEvent (String.format("RotateTo: %3.0f %3.0f %3.0f\n", odo_x_pos_cm(), odo_y_pos_cm(), finalHeading));
            } catch (IOException e) {
                e.printStackTrace();
            }
            return;
        }

        if (Thread.interrupted()) return;
        double iniHeading = odo_heading();
        if (power <= chassisAligmentPower || Math.abs(iniHeading-finalHeading)<5.0) {
            // rawRotateTo(power, finalHeading, false, timeout_sec);//was power
            if (Thread.interrupted()) return;
            if (autoDriveMode== AutoDriveMode.CONTINUE_NO_CORRECTION) return;
            else if (autoDriveMode== AutoDriveMode.STOP_NO_CORRECTION) {
                stop();
                return;
            }
            if (power > chassisAligmentPower)
                rawRotateTo(chassisAligmentPower, finalHeading, false, timeout_sec);
            return;
        }
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
        double lowPowerDegree = 8 + (power - chassisAligmentPower) * 60;

        //ensure the condition before calling rotate()
        useScalePower = false;
        //power the wheels
        if (Thread.interrupted()) return;
        turn(1, direction * power);
        double currentHeading;
        double crossProduct;
        double currentAbsDiff;
        boolean lowerPowerApplied = false;
        long iniTime = System.currentTimeMillis();
        int loop = 0;
        double init_loop_time = System.currentTimeMillis();
        while (true) {
            if (Thread.interrupted()) return;
            currentHeading = odo_heading();
//            info("RotateTo-%.1f, heading =%.3f, pw=%.2f(%s)", finalHeading,currentHeading,power,(lowerPowerApplied?"low":"hi"));
            crossProduct = cross(-currentHeading, -finalHeading);
            //break if target reached or exceeded
            if (direction == -1) {//rotating ccw
                if (crossProduct <= 0) break;
            } else {//rotating cw
                if (crossProduct >= 0) break;
            }
            currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
            if (changePower && !lowerPowerApplied && currentAbsDiff <= lowPowerDegree) {//damp power to alignment power if in last 40%, (currentAbsDiff / iniAbsDiff < 0.40)
                if (Thread.interrupted()) return;
                if (odo_speed_cm()>20) {
                    turn(1, 0.0);
                    sleep(40);
                    turn(1, direction * (chassisAligmentPower - 0.02));
                    lowerPowerApplied = true;
                }
            }
            if (currentAbsDiff / iniAbsDiff < 0.20 && abs(crossProduct) * radToDegree < 1.0)//assume sinx=x, stop 1 degree early
                break;//stop if really close to target
            if (Thread.interrupted()) break;
            if (System.currentTimeMillis() - iniTime > timeout_sec*1000) break;
            TaskManager.processTasks();
            loop++;
        }
        if (Thread.interrupted()) return;
        double end_loop_time = System.currentTimeMillis();
        if (loop>0)
            auto_loop_time = (end_loop_time-init_loop_time)/(double)loop;

        stop();
        if (!finalCorrection) {
            teleOpDriveMode = TeleOpDriveMode.STOP;
            useScalePower = true;
            return;
        }
        if (Thread.interrupted()) return;
        if (odo_speed_cm()>10)
            sleep(90);

        //**************Check for overshoot and correction**************
        if (autoDriveMode== AutoDriveMode.STOP||autoDriveMode== AutoDriveMode.CONTINUE) {
            currentHeading = odo_heading();
            currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
            if ((currentAbsDiff > 1) && !Thread.interrupted()) {
                rawRotateTo(chassisAligmentPower, finalHeading, false, 0.5);
            }
        }
        if (autoDriveMode== AutoDriveMode.STOP_NO_CORRECTION || autoDriveMode== AutoDriveMode.STOP) {
            stop();
        }
        if (Thread.interrupted()) return;
        //**************End correction**************
        teleOpDriveMode = TeleOpDriveMode.STOP;
        useScalePower = true;
    }



    //cross two unit vectors whose argument angle is given in degree
    public static double cross(double theta, double phi) {
        return cos(theta * degreeToRad) * sin(phi * degreeToRad) - sin(theta * degreeToRad) * cos(phi * degreeToRad);
    }

    public void updateInitPosFromOdo(){
        initX = odo_x_pos_cm();
        initY = odo_y_pos_cm();
    }

    public void initOdoFromJson(){
        set_init_pos(initX, initY, odo_heading());
    }
}
