package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;

@Deprecated
public class StateMoveToWarehouseY implements EbotsAutonState{

    StopWatch stopWatch = new StopWatch();
    EbotsAutonOpMode autonOpMode;

    private String name = this.getClass().getSimpleName();

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private ArrayList<DcMotorEx> motors = new ArrayList<>();
    private double speed;
    private long driveTime;

    private DistanceSensor distanceSensor;
    private double targetDistance;
    private RobotSide travelDirection;
    private long stateTimeLimit=1250;



    public StateMoveToWarehouseY(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backRight);
        motors.add(backLeft);

        stopWatch.reset();

        if(autonOpMode.getAlliance() == Alliance.BLUE){
            distanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
            travelDirection = RobotSide.LEFT;

        } else {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
            travelDirection = RobotSide.RIGHT;
        }

        if(autonOpMode.getStartingSide() == StartingSide.CAROUSEL){
            targetDistance = 6;
        } else{
            targetDistance = 3;
        }
    }


    @Override
    public boolean shouldExit() {
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean targetPositionAchieved = currentDistance <= targetDistance;

        boolean shouldExit = stateTimedOut | targetPositionAchieved | !autonOpMode.opModeIsActive();

        return shouldExit;

    }

    @Override
    public void performStateActions() {
        if (travelDirection == RobotSide.RIGHT) {
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            frontLeft.setPower(+speed);
            backRight.setPower(+speed);
        } else{
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            frontLeft.setPower(-speed);
            backRight.setPower(-speed);
        }
    }

    @Override
    public void performTransitionalActions() {
            for(DcMotorEx motor: motors) {
                motor.setPower(0.0);
            }

    }
}
