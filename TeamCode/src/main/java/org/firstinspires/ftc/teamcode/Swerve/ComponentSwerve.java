package org.firstinspires.ftc.teamcode.Swerve;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class ComponentSwerve {
    /* Declare OpMode members. */
    private LinearOpMode myOp = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    protected DcMotorEx lfDrive = null;
    protected DcMotorEx rfDrive = null;
    protected DcMotorEx lrDrive = null;
    protected DcMotorEx rrDrive = null;

    private Servo lfTurn = null;
    private Servo rfTurn = null;
    private Servo lrTurn = null;
    private Servo rrTurn = null;

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    // Swerve chassis constants
    protected static final double wDia = 96.0; // mm
    protected static final double wCir = wDia * 3.141592; // 301.593 wheel circumference
    protected static final double mEnc = 537.7; // PPR
    protected static final double mRPM = 312; // RPM 5.2 rps
    protected static final double cRadius = 203.2;// chassis radius mm from center for turning
    protected static final double gearRatio = 1.7; // gear ratio

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    protected static final double lfFWD = 0.0; // each servo is calibrated for movement
    protected static final double rfFWD = 0.0;
    protected static final double lrFWD = 0.0;
    protected static final double rrFWD = 0.0;

    protected boolean drvTrn = true; // if true drive straight or false wheels 45 deg turn
    protected int encStartAvg = 0; // start encoder drive count
    protected int enclfFWD = 0; // start encoder drive count
    protected int encrfFWD = 0;
    protected int enclrFWD = 0;
    protected int encrrFWD = 0;
    protected static final double degInc = 0.02;  // sets Servo turn degrees
    protected static final double degSlope = 0.005556; // (outend - outstrt)/(inpend - inpstrt)

    protected static double xorg = 0.0; // last position start to next point
    protected static double yorg = 0.0;

    protected static double xobj = 0.0; // obj avoidance position start to next point
    protected static double yobj = 0.0;

    protected static double xdes = 0.0; // destination position start to next point
    protected static double ydes = 0.0;

    public static double desV = 0.0; // distance to travel
    public static double angV = 0.0; // angle to turn
    public static double drvSpd = 0.2; // saved drive speed to pass to turn and drive
    protected static int ltarget = 0;
    protected static int rtarget = 0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ComponentSwerve(LinearOpMode opmode) {
        myOp = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        lfDrive = myOp.hardwareMap.get(DcMotorEx.class, "Lfront");
        lrDrive = myOp.hardwareMap.get(DcMotorEx.class, "Lrear");
        rfDrive = myOp.hardwareMap.get(DcMotorEx.class, "Rfront");
        rrDrive = myOp.hardwareMap.get(DcMotorEx.class, "Rrear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
        lrDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rfDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rrDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        lfDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lrDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        lfTurn = myOp.hardwareMap.get(Servo.class, "LTfront");
        lrTurn = myOp.hardwareMap.get(Servo.class, "LTrear");
        rfTurn = myOp.hardwareMap.get(Servo.class, "RTfront");
        rrTurn = myOp.hardwareMap.get(Servo.class, "RTrear");

        // Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = myOp.hardwareMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the AUTO Bulk Caching mode
        // MANUAL for Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        setMtrDir(); // true set drive wheels to drive position

        navxMicro = myOp.hardwareMap.get(NavxMicroNavigationSensor.class, "navX");
        gyro = navxMicro;

        timer.reset();
        while (navxMicro.isCalibrating()) {
            myOp.idle();
        }

        myOp.telemetry.addData(">", "Hardware Initialized");
        myOp.telemetry.update();
    }

    // calculate distance traveled in encoder pulses
    protected int encCntD(int dist) { // encoder count distance
        return (int) (mEnc * dist / wCir);
    }

    // calculate distance traveled in encoder pulses as whole pulse; integer
    protected int encCntR(int angle) { // encoder count radius turning
        return (int) (gearRatio * mEnc * angle / 360.0);
    }

    private void driveSpeed(double desV) {
        // Use existing function to drive wheels.
        lfDrive.setPower(desV);
        rfDrive.setPower(desV);
        lrDrive.setPower(desV);
        rrDrive.setPower(desV);
    }

    private void setMtrDir() { // DT true DRIVE else Set TURN
        // set to max time to move servo into position
        // for 5 Turn Torque Gobilda servo at 6v No load .2 sec per 60 deg.
        // servo to wheel gear ratio ~  1 to 3
        // calc no load max 1.8 Sec 180 deg. min .02 Sec 2 deg.
        // 2.5 S to consider friction of wheel and weight on it
        if (!drvTrn) { // Turn
            lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
            lrDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rfDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rrDrive.setDirection(DcMotorEx.Direction.FORWARD);
            lfTurn.setPosition(.75);// offset calibration
            rrTurn.setPosition(.25);
            rfTurn.setPosition(0.25);// wheel at -45 deg
            lrTurn.setPosition(.75);// wheel at 45 deg
            myOp.sleep(1500);// wait for action
        }
        if (drvTrn) { // drive
            lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
            lrDrive.setDirection(DcMotorEx.Direction.REVERSE);
            rfDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rrDrive.setDirection(DcMotorEx.Direction.REVERSE);
            lfTurn.setPosition(0.5); // set to drive forward
            lrTurn.setPosition(0.5);
            rfTurn.setPosition(0.5);
            rrTurn.setPosition(0.5);
            myOp.sleep(1500);// wait for action
        }
    }

    /* value of drv speed determining rules
       forward speed is -1 to 1
       if > 2 then degree assumed strafing mode < 44 degree
       turning mode > 44
       add update XY position
    */
    protected void driveRobot(double drvSpd) {

        if (Math.abs(drvSpd) > 44) {
            turnRobot((int) drvSpd);
        } // if drvSpd > 40
        else if (Math.abs(drvSpd) > 2) {
            strafRobot((int) drvSpd);
        }// if drvSpd > 2
        else if (drvSpd >= 0.2) {
            driveSpeed(drvSpd);
        } else if (drvSpd < -0.2) {
            driveSpeed(-drvSpd);
        }// drvSpd +/- > .2
        else if (Math.abs(drvSpd) < 0.2) {
            driveSpeed(0.0);
        }
    } // end driveRobot

    protected void strafRobot(int turn) {
        // turn is in degrees
        double servoVal = degSlope * (turn + 90);// converts degree into a servo range.

        lfTurn.setPosition(servoVal);// offset calibration
        rfTurn.setPosition(servoVal);
        lrTurn.setPosition(servoVal);
        rrTurn.setPosition(servoVal);
        // set to max time to move servo into position
        // calc no load max 1.8 S min .02 S
        myOp.sleep(33 * Math.abs(turn));// calc delay mS 33 for 5T torque
        if (turn > 0) driveSpeed(0.3);
        if (turn < 0) driveSpeed(-.3);

    }

    protected void turnRobot(int turn) {
        angV = turn;

        int encCnt = encCntR(Math.abs(turn));
        if (turn < 0) encCnt *= -1;// neg deg makes neg enc count
        // check closest turn angle
        // turn is in degrees with IMU degrees
        if (drvTrn) {
            drvTrn = false;
            setMtrDir(); // Set motor directions for turning
        }
        // turn to degree using wheel diameter
        ltarget = lfDrive.getCurrentPosition() + encCnt;
        lfDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lfDrive.setTargetPosition(ltarget);
        rrDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rtarget = rrDrive.getCurrentPosition() + encCnt;
        rrDrive.setTargetPosition(rtarget);
        // set turn drive speed
        if (turn > 0) driveSpeed(0.3);
        else driveSpeed(-0.3);
        while ((lfDrive.isBusy() && rrDrive.isBusy()) && myOp.opModeIsActive()) {
            myOp.idle();
        }
        lfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        drvTrn = true;
        setMtrDir(); // Set motor directions for turning
    }

    /* AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
     double xRate = rates.xRotationRate;
     double yRate = rates.yRotationRate;
     double zRate = rates.zRotationRate;
     */
    // heading X angle from IMU
    public double getXangle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // roll yAngle
    public double getYangle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    // pitch
    public double getZangle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    public double getWDist(double tDist) {
        double vecDist = 0.0;
        return wCir * tDist;
    }

    public double getVecLen(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }

    public double getVecAng(double x, double y) {
        if (x == 0.0) return 0.0;
        return Math.atan(y / x);
    }

    //  (r,θ), write x=rcosθ and y=rsinθ.
    public double getX(double d, double a) {
        return d * Math.cos(a);
    }

    public double getY(double d, double a) {
        return d * Math.sin(a);
    }

    void debugParameters() {
        //Send telemetry messages to explain controls and show drive status
        myOp.telemetry.addData("Drive", "Left Y Stick");
        myOp.telemetry.addData("Turn ", "Left X Stick");
        myOp.telemetry.addData("Called Drive Power", drvSpd);
        myOp.telemetry.addData("Called Turn degree", angV);
        myOp.telemetry.addData("drive/turn", drvTrn);
        myOp.telemetry.addLine("Encoders -");
        myOp.telemetry.addData("ltarget", ltarget);
        myOp.telemetry.addData("lf", lfDrive.getCurrentPosition());
        myOp.telemetry.addData("rf", rfDrive.getCurrentPosition());
        myOp.telemetry.addData("rtarget", rtarget);
        myOp.telemetry.addData("rr", rrDrive.getCurrentPosition());
        myOp.telemetry.addData("lr", lrDrive.getCurrentPosition());
        myOp.telemetry.addLine("NavX -");
        myOp.telemetry.addData("X angle", getXangle());
        myOp.telemetry.addData("Y angle", getYangle());
        myOp.telemetry.addData("Z angle", getZangle());
        myOp.telemetry.update();
    }
}  // end Component

