package org.firstinspires.ftc.teamcode.Swerve;


import static com.qualcomm.robotcore.hardware.DcMotor.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

import java.util.List;

public class ComponentSwerve {

    /* Declare OpMode members. */
    private LinearOpMode myOp = null;   // gain access to methods in the calling OpMode.


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    DcMotor FLMotor, FRMotor, BLMotor, BRMotor;

    Servo FLServo, FRServo, BLServo, BRServo;

    GoBildaPinpointDriver odo;


    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();


    // Swerve chassis constants
    final double wDia = 96.0; // mm
    final double wCir = wDia * 3.141592; // 301.593 wheel circumference
    final double mEnc = 537.7; // PPR
    final double mRPM = 312; // RPM 5.2 rps
    final double cRadius = 203.2;// chassis radius mm from center for turning
    final double gearRatio = 1.7; // gear ratio


    protected boolean driveTurn = true; // if true drive straight or false wheels 45 deg turn
    protected static final double degInc = 0.02;  // sets Servo turn degrees
    protected static final double degSlope = 0.005556; // (outend - outstrt)/(inpend - inpstrt)


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
    public void initRobot() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLMotor = myOp.hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = myOp.hardwareMap.get(DcMotor.class, "BLMotor");
        FRMotor = myOp.hardwareMap.get(DcMotor.class, "FRMotor");
        BRMotor = myOp.hardwareMap.get(DcMotor.class, "BRMotor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(Direction.FORWARD);
        BLMotor.setDirection(Direction.REVERSE);
        FRMotor.setDirection(Direction.FORWARD);
        BRMotor.setDirection(Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        FLServo = myOp.hardwareMap.get(Servo.class, "FLServo");
        BLServo = myOp.hardwareMap.get(Servo.class, "BLServo");
        FRServo = myOp.hardwareMap.get(Servo.class, "FRServo");
        BRServo = myOp.hardwareMap.get(Servo.class, "BRServo");


        // Define and initialize GoBilda Pinpoint driver
        odo = myOp.hardwareMap.get(GoBildaPinpointDriver.class, "ODO");
        odo.setOffsets(0, 0); // Sets the offset of the pod relative to the center of rotation of the robot
        odo.setEncoderResolution(goBILDA_4_BAR_POD);
        odo.setEncoderDirections(FORWARD, FORWARD);


        // All hardware initialized
        myOp.telemetry.addData(">", "Hardware Initialized");
        myOp.telemetry.update();
    }


    /**
     * Calculates the distance traveled in encoder pulses as a whole pulse<br>
     * Integer
     */
    protected int encCntR(int angle) { // encoder count radius turning
        return (int) (gearRatio * mEnc * angle / 360.0);
    }


    /**
     * Sets the power to all the motors the same
     *
     * @param desV The power to run the motors at.<br>
     *             Range = -1.0 to 1.0
     */
    private void driveSpeed(double desV) {
        // Use existing function to drive wheels.
        FLMotor.setPower(desV);
        FRMotor.setPower(desV);
        BLMotor.setPower(desV);
        BRMotor.setPower(desV);
    }


    /**
     * Set to max time to move servo into position
     * for 5 Turn Torque Gobilda servo at 6v No load .2 sec per 60 deg.
     * servo to wheel gear ratio ~  1 to 3
     * calc no load max 1.8 Sec 180 deg. min .02 Sec 2 deg.
     * 2.5 S to consider friction of wheel and weight on it
     */
    private void setMotorDirection() { // DT true DRIVE else Set TURN

        if (!driveTurn) { // Turn
            FLMotor.setDirection(Direction.FORWARD);
            BLMotor.setDirection(Direction.FORWARD);
            FRMotor.setDirection(Direction.FORWARD);
            BRMotor.setDirection(Direction.FORWARD);
            FLServo.setPosition(.75);// offset calibration
            BRServo.setPosition(.25);
            FRServo.setPosition(.25);// wheel at -45 deg
            BLServo.setPosition(.75);// wheel at 45 deg
            myOp.sleep(1500);// Wait for servos to reach position
        }
        if (driveTurn) { // drive
            FLMotor.setDirection(Direction.FORWARD);
            BLMotor.setDirection(REVERSE);
            FRMotor.setDirection(Direction.FORWARD);
            BRMotor.setDirection(REVERSE);
            FLServo.setPosition(0.5); // set to drive forward
            BLServo.setPosition(0.5);
            FRServo.setPosition(0.5);
            BRServo.setPosition(0.5);
            myOp.sleep(1500);// Wait for servos to reach position
        }

    }


    /**
     * Value of drv speed determining rules
     * Speed is -1 to 1
     * If > 2 then degree assumed strafing mode < 44 degree
     * turning mode > 44
     * add update XY position
     */
    protected void driveRobot(double drvSpd) {

        if (Math.abs(drvSpd) > 44) {
            turnRobot((int) drvSpd);
        } // if drvSpd > 40
        else if (Math.abs(drvSpd) > 2) {
            strafeRobot((int) drvSpd);
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


    /**
     * ?
     */
    protected void strafeRobot(int turn) {
        // turn it into degrees
        double servoVal = degSlope * (turn + 90);// converts degree into a servo range.

        FLServo.setPosition(servoVal);// offset calibration
        FRServo.setPosition(servoVal);
        BLServo.setPosition(servoVal);
        BRServo.setPosition(servoVal);

        // set to max time to move servo into position
        // calc no load max 1.8 S min .02 S
        myOp.sleep((long) 33 * Math.abs(turn));// calc delay mS 33 for 5T torque
        if (turn > 0) driveSpeed(0.3);
        if (turn < 0) driveSpeed(-0.3);

    }


    protected void turnRobot(int turn) {
        angV = turn;

        int encCnt = encCntR(Math.abs(turn));
        if (turn < 0) encCnt *= -1;// neg deg makes neg enc count
        // check closest turn angle
        // turn is in degrees with IMU degrees
        if (driveTurn) {
            driveTurn = false;
            setMotorDirection(); // Set motor directions for turning
        }
        // turn to degree using wheel diameter
        ltarget = FLMotor.getCurrentPosition() + encCnt;
        FLMotor.setMode(RUN_USING_ENCODER);
        FLMotor.setTargetPosition(ltarget);
        BRMotor.setMode(RUN_USING_ENCODER);
        rtarget = BRMotor.getCurrentPosition() + encCnt;
        BRMotor.setTargetPosition(rtarget);
        // set turn drive speed
        if (turn > 0) driveSpeed(0.3);
        else driveSpeed(-0.3);
        while ((FLMotor.isBusy() && BRMotor.isBusy()) && myOp.opModeIsActive()) {
            myOp.idle();
        }
        FLMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        driveTurn = true;
        setMotorDirection(); // Set motor directions for turning
    }


    /**
     * Prints a lot of info to the DriverStation
     */
    void debugParameters() {
        //Send telemetry messages to explain controls and show drive status
        myOp.telemetry.addData("Drive", "Left Y Stick");
        myOp.telemetry.addData("Turn ", "Left X Stick");
        myOp.telemetry.addData("Called Drive Power", drvSpd);
        myOp.telemetry.addData("Called Turn degree", angV);
        myOp.telemetry.addData("drive/turn", driveTurn);

        myOp.telemetry.addLine("Encoders -");
        myOp.telemetry.addData("Ltarget", ltarget);
        myOp.telemetry.addData("lf", FLMotor.getCurrentPosition());
        myOp.telemetry.addData("rf", FRMotor.getCurrentPosition());
        myOp.telemetry.addData("Rtarget", rtarget);
        myOp.telemetry.addData("rr", BRMotor.getCurrentPosition());
        myOp.telemetry.addData("lr", BLMotor.getCurrentPosition());

        myOp.telemetry.addLine("NavX -");
        myOp.telemetry.addData("X angle", 0);
        myOp.telemetry.addData("Y angle", 0);
        myOp.telemetry.addData("Z angle", 0);
        myOp.telemetry.update();
    }
}  // end Component

