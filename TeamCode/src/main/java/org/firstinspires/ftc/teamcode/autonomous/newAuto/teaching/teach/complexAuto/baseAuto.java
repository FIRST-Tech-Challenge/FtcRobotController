package org.firstinspires.ftc.teamcode.autonomous.newAuto.teaching.teach.complexAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class baseAuto extends LinearOpMode implements baseAutoInterface {
        baseHardware hardware = new baseHardware();
        int tick_per_inch = 0;
        int tick_per_360 = 0;
        @Override
        public void runOpMode() throws InterruptedException {
            initialize();
            setDirection();
            while(opModeIsActive()){
                movement(baseAutoEnum.FORWARD, 12, 0, 0.5);
                sleep(1000);
            }
        }
    public void initialize() {
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
        hardware.lift = hardwareMap.get(DcMotor.class, "lift");
        hardware.claw = hardwareMap.get(CRServo.class, "claw");
    }
    public void setDirection() {
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);

    }
    public void setMode(){
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR) {
        hardware.frontLeft.setTargetPosition(targetPosFL);
        hardware.frontRight.setTargetPosition(targetPosFR);
        hardware.backLeft.setTargetPosition(targetPosBL);
        hardware.backRight.setTargetPosition(targetPosBR);
        setMode();
        hardware.frontLeft.setPower(speedFL);
        hardware.frontRight.setPower(speedFR);
        hardware.backLeft.setPower(speedBL);
        hardware.backRight.setPower(speedBR);
    }
    public void movement(baseAutoEnum state, double ticks, double rotation, double speed){
        int targetPos = (int) (ticks * tick_per_inch);
        int targetRotation = (int) (rotation * tick_per_360);
        switch(state){
            case FORWARD:
                base(targetPos, targetPos, targetPos, targetPos, speed, speed, speed, speed);
                break;
            case BACKWARD:
                base(-targetPos, -targetPos, -targetPos, -targetPos, -speed, -speed, -speed, -speed);
                break;
            case STRAFE_LEFT:
                base(-targetPos, targetPos, targetPos, -targetPos, -speed, speed, speed, -speed);
                break;
            case STRAFE_RIGHT:
                base(targetPos, -targetPos, -targetPos, targetPos, speed, -speed, -speed, speed);
                break;
            case TURN_LEFT:
                base(-targetRotation, -targetRotation, targetRotation, targetRotation, -speed, -speed, speed, speed);
                break;
            case TURN_RIGHT:
                base(targetRotation, targetRotation, -targetRotation, -targetRotation, speed, speed, -speed, -speed);
                break;
        }
    }
}
