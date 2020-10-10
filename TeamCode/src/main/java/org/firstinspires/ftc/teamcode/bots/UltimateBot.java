package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

import java.util.Random;

public class UltimateBot extends YellowBot
{
    private static int SWINGVALUE = 280;
    private static int MIDDLESWINGVALUE = 200;
    public DcMotor wobbleSwing = null;

    HardwareMap hwMap =  null;

    private Servo wobbleClaw = null;

    /* Constructor */
    public UltimateBot(){

    }
    @Override
    public void init(LinearOpMode owner, HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        super.init(owner, ahwMap, telemetry);
        hwMap = ahwMap;

        try {
            wobbleSwing = hwMap.get(DcMotor.class, "swing");
            wobbleSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleSwing.setDirection(DcMotor.Direction.FORWARD);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleSwing.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues wobbleSwing. Check the controller config", ex);
        }



        try{
            wobbleClaw = hwMap.get(Servo.class, "claw");
            wobbleClaw.setPosition(1);
        }
        catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw. Check the controller config", ex);
        }


        telemetry.addData("Init", "Ultimate is ready");
    }


    public int moveWobbleSwing(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        wobbleSwing.setPower(power);
        int position = wobbleSwing.getCurrentPosition();
        return position;
    }


    public void moveWobbleClaw (double position) {
        double p = Range.clip(position, -1.0, 1.0);

        wobbleClaw.setPosition(p);

    }


    @BotAction(displayName = "closeWobbleClaw")
    public void closeWobbleClaw () {

        wobbleClaw.setPosition(0);
    }

    @BotAction(displayName = "openWobbleClaw")
    public void openWobbleClaw () {

        wobbleClaw.setPosition(1);
    }

    @BotAction(displayName = "backWobbleSwing")
    public void backWobbleSwing (){
        int currentPosition = wobbleSwing.getCurrentPosition();
        int targetPosition = currentPosition - SWINGVALUE;
        if (targetPosition <= -50) {
            return;
        }
        wobbleSwing.setTargetPosition(targetPosition);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.3);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        wobbleSwing.setPower(0);
    }

    @BotAction(displayName = "forwardWobbleSwing")
    public void forwardWobbleSwing () {
        int currentPosition = wobbleSwing.getCurrentPosition();
        int targetPosition = currentPosition + SWINGVALUE;
        if (targetPosition >= SWINGVALUE + 50) {
            return;
        }
        wobbleSwing.setTargetPosition(targetPosition);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.3);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        wobbleSwing.setPower(0);
    }

    @BotAction(displayName = "middleWobbleSwing")
    public void middleWobbleSwing() {
        wobbleSwing.setTargetPosition(MIDDLESWINGVALUE);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.3);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        wobbleSwing.setPower(0);
    }

    //actions
    @BotAction(displayName = "signalOK")
    public void signalOK(){
        getLights().OK();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1){

        }
        getLights().none();
    }

    @BotAction(displayName = "signalProblem")
    public void signalProblem(){
        getLights().problem();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1){

        }
        getLights().none();
    }

    @BotAction(displayName = "detectStack")
    public AutoDot detectStack(){
        AutoDot target = new AutoDot();
        RingDetector rf = null;
        rf = new RingDetector(this.hwMap, telemetry);
        rf.initDetector();
        target = rf.detectRing(2, telemetry, owner);
        signalOK();
        return target;
    }

}
