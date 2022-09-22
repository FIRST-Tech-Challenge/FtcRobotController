package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;

@Deprecated
public class StateDriveToCarousel implements EbotsAutonState{
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Class Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Instance Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    StopWatch stopWatch;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    ArrayList<DcMotorEx> motors = new ArrayList<>();

    DistanceSensor distanceSensor;
    double currentDistance;
    double targetDistance;

    private long stateTimeLimit=750;
    private EbotsAutonOpMode autonOpMode;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateDriveToCarousel(EbotsAutonOpMode autonOpMode) {
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        this.autonOpMode = autonOpMode;
        stopWatch = new StopWatch();
        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontRight);
        motors.add(frontLeft);
        motors.add(backRight);
        motors.add(backLeft);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");
        targetDistance = 10;
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
        currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean targetPositionAchieved = currentDistance <= targetDistance;
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;


        return targetPositionAchieved | stateTimedOut | !autonOpMode.opModeIsActive();

    }

    @Override
    public void performStateActions() {

        double currentError = currentDistance - targetDistance;
        double power = currentError / 20;
        double maxAllowedSpeed = 0.6;

        // Now make sure that the power doesn't exceed our maxAllowedSpeed, and preserve sign
        double powerSign = Math.signum(power);
        double powerMagnitude = Math.min(Math.abs(power), maxAllowedSpeed);
        power = powerSign * powerMagnitude;

        for(DcMotorEx m : motors) {
            m.setPower(-power);
        }
    }

    @Override
    public void performTransitionalActions() {
        for (DcMotorEx m : motors) {
            m.setPower(0.0);
        }
    }

}
