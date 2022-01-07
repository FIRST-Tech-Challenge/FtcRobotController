package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TacoAuto1", preselectTeleOp = "TacoLights")

public class TacoAuto1 extends Taco_FF_Super_Class {

    @Override
    public void runOpMode() {
        initialization(true);
        Spot();
        waitForStart();
        if (opModeIsActive()) {
            if (blue && far && spin){
                blueduck();
            } else if(far && blue) {
                bluelame();
            } else if (blue){
                blueshared();
            } else if (far && spin){
                redduck();
            } else if (far){
                telemetry.addLine("Running redlame()");
                telemetry.update();
                redlame();
            } else {
                redshared();
            }
        }
    }

    private void bluelame() {
        if (Delay){
            telemetry.addLine("Delayed");
            telemetry.addLine("Running bluelame()");
            telemetry.update();
            sleep(25000);
        } else {
            telemetry.addLine("No delay");
            telemetry.addLine("Running bluelame()");
            telemetry.update();
        }
        go_same(1200, .5, 2000);
    }

    private void blueish() {
        telemetry.addLine("Running blueish()");
        telemetry.update();
        go_same(-50, .3, 500);
        Duck.setPower(.2);
        sleep(4000);
        go_same(200, .5, 1000);
        go_diff(-70, -70, 70, 700, .50, 2000);
        go_diff(1000, 1000, 1000, 1000, .75, 2000);
    }

    private void redlame() {
        if (Delay){
            telemetry.addLine("Delayed");
            sleep(25000);
        } else {
            telemetry.addLine("No delay");
        }
        go_same(1200, .5, 2000);
    }

    private void redduck(){
        if (Delay){
            telemetry.addLine("Delayed");
            telemetry.addLine("Running redduck()");
            telemetry.update();
            sleep(15000);
        } else {
            telemetry.addLine("No delay");
            telemetry.addLine("Running redduck()");
            telemetry.update();
        }
        go_same(-100, .2, 1000);
        Duck.setPower(-.3);
        sleep(3000);
        go_diff(100, 100, -200, -200, .5, 2000);
        arm_target = 400;
        Arm.setTargetPosition(arm_target);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(-.5);
        Duck.setPower(0);
        go_same(5000, 1, 4000);
    }

    private void blueduck(){
            telemetry.addLine("Running blueduck()");
            telemetry.update();
            go_diff(-175, -175, 175, 175, .3, 1500);
            Duck.setPower(.3);
            sleep(3000);
            Duck.setPower(0);
            if (Delay){
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(96));
                telemetry.addLine("Delayed");
                telemetry.update();
                sleep(15000);
            } else {
                telemetry.addLine("Running blueduck()");
                telemetry.update();
            }
            telemetry.addLine("Running blueduck()");
            telemetry.update();
            Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(42));
            go_same(650, .5, 1000);
            go_diff(-610, -610, 610, 610, .5, 2000);
            arm_target = 300;
            Arm.setTargetPosition(arm_target);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(-.3);
            Duck.setPower(0);
            go_same(-5250, .75, 3000);
    }

    private void redshared(){
        if (Delay){
            telemetry.addLine("Delayed");
            telemetry.addLine("Running redshared()");
            telemetry.update();
            sleep(20000);
        } else {
            telemetry.addLine("No delay");
            telemetry.addLine("Running redshared()");
            telemetry.update();
        }
        arm_target = 400;
        Arm.setTargetPosition(arm_target);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(-.5);
        sleep(2000);
        go_same(-3000, -1, 2000);
        arm_target = 0;
        Arm.setTargetPosition(arm_target);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(.2);
        sleep(2000);
    }

    private void blueshared(){
        if (Delay){
            telemetry.addLine("Delayed");
            telemetry.addLine("Running blueshared()");
            telemetry.update();
            sleep(20000);
        } else {
            telemetry.addLine("No delay");
            telemetry.addLine("Running blueshared()");
            telemetry.update();
        }
        arm_target = 400;
        Arm.setTargetPosition(arm_target);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(-.5);
        sleep(2000);
        go_same(-3000, -1, 2000);
        arm_target = 0;
        Arm.setTargetPosition(arm_target);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(.2);
        sleep(2000);
    }
}
