package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;
@Deprecated
public class StateRotateNinetyDegrees implements EbotsAutonState{


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
    private EbotsAutonOpMode autonOpMode;
    // State used for updating telemetry
    private double targetHeadingDeg;
    private double currentHeading;
    ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
    ArrayList<DcMotorEx> rightMotors = new ArrayList<>();
    long stateTimeLimit = 2000;
    StopWatch stopWatch = new StopWatch();
    private double currentError;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateRotateNinetyDegrees(EbotsAutonOpMode autonOpMode){
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        this.autonOpMode = autonOpMode;

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
        if (autonOpMode.getAlliance() == Alliance.BLUE){
            targetHeadingDeg = 90;
        } else {
            targetHeadingDeg = -90;
        }
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
        double acceptableError = 3;
        boolean targetHeadingAchieved = false;

        currentHeading = EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true);
        currentError = currentHeading - targetHeadingDeg;
        currentError = UtilFuncs.applyAngleBounds(currentError);

        if (Math.abs(currentError)  <= acceptableError){
            targetHeadingAchieved = true;
        }

        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        return targetHeadingAchieved | stateTimedOut | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        double power = currentError * 0.01;

        for (DcMotorEx m : leftMotors) {
            m.setPower(power);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(-power);
        }
//        } else {
//            for(DcMotorEx m : leftMotors) {
//                m.setPower(-power);
//            }
//            for(DcMotorEx m : rightMotors) {
//                m.setPower(power);
//            }
//        }
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