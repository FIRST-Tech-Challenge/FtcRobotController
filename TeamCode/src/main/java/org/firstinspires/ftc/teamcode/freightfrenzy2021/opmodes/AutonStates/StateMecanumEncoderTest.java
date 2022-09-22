package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.MecanumWheel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

import java.util.ArrayList;

public class StateMecanumEncoderTest implements EbotsAutonState{

    StopWatch stopWatch = new StopWatch();
    EbotsAutonOpMode autonOpMode;

    private String name = this.getClass().getSimpleName();

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    //didn't know what else to call them
    private ArrayList<DcMotorEx> motors = new ArrayList<>();
    private ArrayList<DcMotorEx> mecanumMotors = new ArrayList<>();
    private double speed;
    private long driveTime;


    public StateMecanumEncoderTest(EbotsAutonOpMode autonOpMode, MecanumWheel mecanumWheel){
        mecanumWheel.zeroMotorEncoders();

        this.autonOpMode = autonOpMode;
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        mecanumMotors.add(frontRight);
        motors.add(backRight);
        mecanumMotors.add(backLeft);

        stopWatch.reset();

        if(autonOpMode.getStartingSide() == StartingSide.CAROUSEL){
            driveTime = 850;
            speed = 1.0;
        } else {
            driveTime = 200;
            speed = -1.0;
        }


    }

    @Override
    public boolean shouldExit() {

        boolean shouldExit = false;

        if (autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper){
            shouldExit = true;
        }

        autonOpMode.telemetry.update();
        return shouldExit | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        if(stopWatch.getElapsedTimeMillis() <= driveTime){
            for(DcMotorEx motor: motors) {
                motor.setPower(speed);
            }
            for(DcMotorEx motor: mecanumMotors) {
                motor.setPower(-speed);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
            for (DcMotorEx motor : mecanumMotors) {
                motor.setPower(0);
            }
        }
    }

    @Override
    public void performTransitionalActions() {
        for(DcMotorEx motor: motors) {
            motor.setPower(0.0);
        }
        for (DcMotorEx motor : mecanumMotors) {
            motor.setPower(0);
        }

    }
    public void updateTelemetry(){
        int i = 0;
        int m = 0;
        for (DcMotorEx motor : motors){
            autonOpMode.telemetry.addData("fl, br, motor " + String.format("%d", i++), motor.getCurrentPosition());
        }
        for (DcMotorEx motor : motors){
            autonOpMode.telemetry.addData("fr, bl, motor " + String.format("%d", m++), motor.getCurrentPosition());
        }

    }
}
