package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;
@Deprecated
public class StateRotateToZeroDegrees implements EbotsAutonState{


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private EbotsAutonOpMode opMode;
    // State used for updating telemetry
    private Orientation angles;
    private double targetHeadingDeg;
    ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
    ArrayList<DcMotorEx> rightMotors = new ArrayList<>();

    long stateTimeLimit = 2000;
    StopWatch stopWatch = new StopWatch();
    private double currentError;
    private String logTag = "EBOTS";

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateRotateToZeroDegrees(EbotsAutonOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        rightMotors.add(frontRight);
        leftMotors.add(frontLeft);
        rightMotors.add(backRight);
        leftMotors.add(backLeft);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        targetHeadingDeg = 0;
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public boolean shouldExit() {
        boolean targetHeadingAchieved = false;
        double acceptableError = 3;

        currentError = opMode.getCurrentHeadingDeg(true) - targetHeadingDeg;
        currentError = UtilFuncs.applyAngleBounds(currentError);

        if (Math.abs(currentError)  <= acceptableError){
            targetHeadingAchieved = true;
        }
        Log.d(logTag, "Heading achieved: " + targetHeadingAchieved + " Current Error: " + String.format("%.2f", currentError));
        Log.d(logTag, stopWatch.toString());
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        return targetHeadingAchieved  | stateTimedOut | !opMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        // note that this uses getCurrentHeadingDeg function, which is a time-buffered value cache
        // the hardware read occurs during shouldExit
        double power = currentError * 0.01;

        for(DcMotorEx m : leftMotors) {
            m.setPower(power);
        }
        for(DcMotorEx m : rightMotors) {
            m.setPower(-power);
        }
    }

    @Override
    public void performTransitionalActions() {
        for (DcMotorEx m : leftMotors) {
            m.setPower(0.0);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(0.0);
        }
    }
}