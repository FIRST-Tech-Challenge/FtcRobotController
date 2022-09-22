package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.MecanumDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.MecanumWheel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;

public class StateCalibrateHubX implements EbotsAutonState{

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

    DistanceSensor backDistanceSensor;



    public StateCalibrateHubX(EbotsAutonOpMode autonOpMode){
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
        motors.add(backLeft);
        motors.add(backRight);

        stopWatch.reset();

        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");


        if(autonOpMode.getStartingSide() == StartingSide.CAROUSEL){
            driveTime = 0;
            speed = 1.0;
        } else {
            driveTime = 200;
            speed = -1.0;
        }

        for(DcMotorEx motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

      @Override
    public boolean shouldExit() {

        boolean shouldExit = false;
        boolean lockOutActive = stopWatch.getElapsedTimeMillis() < 1000L;
        if ((autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper)
                && !lockOutActive){
            shouldExit = true;
        }

          autonOpMode.telemetry.update();
        return shouldExit | autonOpMode.isStarted() | autonOpMode.isStopRequested();
    }

    @Override
    public void performStateActions() {
        speed = 1.0;
        if(stopWatch.getElapsedTimeMillis() <= driveTime){
            for(DcMotorEx motor: motors) {
                motor.setPower(speed);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
        }
        updateTelemetry();
    }

    @Override
    public void performTransitionalActions() {
            for(DcMotorEx motor: motors) {
                motor.setPower(0.0);
            }
    }
    public void updateTelemetry(){
        int i = 0;
        for (DcMotorEx motor : motors){
            autonOpMode.telemetry.addData("motor " + String.format("%d", i++), motor.getCurrentPosition());
        }

        autonOpMode.telemetry.addData("Back Distance", backDistanceSensor.getDistance(DistanceUnit.INCH));

    }
}
