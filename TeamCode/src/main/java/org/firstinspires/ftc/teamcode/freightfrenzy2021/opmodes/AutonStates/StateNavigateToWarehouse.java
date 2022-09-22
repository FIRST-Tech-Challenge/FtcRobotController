package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;

public class StateNavigateToWarehouse implements EbotsAutonState{

    StopWatch stopWatch;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private ArrayList<DcMotorEx> motors = new ArrayList<>();
    private long driveTime;

    public StateNavigateToWarehouse(EbotsAutonOpMode autonOpMode){
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backRight);
        motors.add(backLeft);
        driveTime = 650;
        stopWatch = new StopWatch();
    }

    @Override
    public boolean shouldExit() {
        boolean shouldExit = false;


        if(stopWatch.getElapsedTimeMillis() >= driveTime){
            shouldExit = true;
        }
        return shouldExit;
    }

    @Override
    public void performStateActions() {
        double speed = 1.0;

        for(DcMotorEx motor: motors){
            motor.setPower(speed);
        }
    }

    @Override
    public void performTransitionalActions() {
        for(DcMotorEx motor: motors){
            motor.setPower(0.0);
        }
    }
}
