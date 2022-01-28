package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
@Disabled
public class Auto_RedCarousel extends LinearOpMode {
    DcMotorEx frontL, frontR, backL, backR, duckWheel, extender, arm = null;

    public void drive(double directionInDegrees, double distanceInCm){
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//        4.27(PPR) = 1 Degree
        final double oneCmInPPR = 7.9;
        final double oneDegreeInPPR = 4.27;
        final double velocity = 500;
        double pprForward = distanceInCm * oneCmInPPR;
        double pprTurn = directionInDegrees * oneDegreeInPPR;


        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                frontL.setTargetPosition(-(int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition((int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition(-(int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition((int) pprTurn + backR.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                frontL.setTargetPosition((int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition(-(int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition((int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition(-(int) pprTurn + backR.getCurrentPosition());
            }

            frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontL.setVelocity(velocity);
            frontR.setVelocity(velocity);
            backR.setVelocity(velocity);
            backL.setVelocity(velocity);

            while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
                telemetry.addData("Status", "Waiting for Motors to Finish Turning");
                telemetry.addData("Motors", "frontL Position: %d, %d", frontL.getCurrentPosition(), frontL.getTargetPosition());
                telemetry.addData("Motors", "frontR Position: %d, %d", frontR.getCurrentPosition(), frontR.getTargetPosition());
                telemetry.addData("Motors", "backL Position: %d, %d", backL.getCurrentPosition(), backL.getTargetPosition());
                telemetry.addData("Motors", "backR Position: %d, %d", backR.getCurrentPosition(), backR.getTargetPosition());
                telemetry.update();
            }
        }

        if(distanceInCm != 0) {
            frontL.setTargetPosition((int) pprForward + frontL.getCurrentPosition());
            frontR.setTargetPosition((int) pprForward + frontR.getCurrentPosition());
            backL.setTargetPosition((int) pprForward + backL.getCurrentPosition());
            backR.setTargetPosition((int) pprForward + backR.getCurrentPosition());

            frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontL.setVelocity(velocity);
            frontR.setVelocity(velocity);
            backR.setVelocity(velocity);
            backL.setVelocity(velocity);

            while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
                telemetry.addData("Status", "Waiting for Motors to Finish Moving");
                telemetry.addData("Motors", "frontL Position: %d, %d", frontL.getCurrentPosition(), frontL.getTargetPosition());
                telemetry.addData("Motors", "frontR Position: %d, %d", frontR.getCurrentPosition(), frontR.getTargetPosition());
                telemetry.addData("Motors", "backL Position: %d, %d", backL.getCurrentPosition(), backL.getTargetPosition());
                telemetry.addData("Motors", "backR Position: %d, %d", backR.getCurrentPosition(), backR.getTargetPosition());
                telemetry.update();
            }
        }
    }

    public void arm(double height, double extension){
        // HEIGHT:
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        duckWheel = hardwareMap.get(DcMotorEx.class, "duckWheel");
        frontL  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        backL  = hardwareMap.get(DcMotorEx.class, "leftRear");
        backR = hardwareMap.get(DcMotorEx.class, "rightRear");
        extender = hardwareMap.get(DcMotorEx.class, "extender");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Reset Encoder
        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);
        extender.setDirection(DcMotorEx.Direction.FORWARD); //TODO: Find correct direction
        arm.setDirection(DcMotorEx.Direction.FORWARD); //TODO: Find correct direction

        waitForStart();
        drive(90, 0);

    }
}
