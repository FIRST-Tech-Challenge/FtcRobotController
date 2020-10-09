package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UltimateBot extends YellowBot
{
    private static int SWINGVALUE = 280;
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


        telemetry.addData("Init", "Dummy is ready");
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


    public void closeWobbleClaw () {
        wobbleClaw.setPosition(0);
    }

    public void openWobbleClaw () {
        wobbleClaw.setPosition(1);
    }

    public void backWobbleSwing (){
        int currentPosition = wobbleSwing.getCurrentPosition();
        int targetPosition = currentPosition - SWINGVALUE;
        wobbleSwing.setTargetPosition(targetPosition);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.3);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        wobbleSwing.setPower(0);
    }

    public void forwardWobbleSwing () {
        int currentPosition = wobbleSwing.getCurrentPosition();
        int targetPosition = currentPosition + SWINGVALUE;
        wobbleSwing.setTargetPosition(targetPosition);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.3);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        wobbleSwing.setPower(0);
    }

}
