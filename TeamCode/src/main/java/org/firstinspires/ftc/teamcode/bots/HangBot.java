package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangBot extends LimelightBot {

    private DcMotorEx hangMotor = null;
    private double hangPower = 1.0;
    private boolean hangReady = false;
    private int hangTarget = 1000;
    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        hangMotor = hwMap.get(DcMotorEx.class, "Hang Motor");
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setPower(hangPower);


    }

    public HangBot(LinearOpMode opMode) {
        super(opMode);
    }

    protected void onTick() {
        super.onTick();}
        public void hang(boolean button){
            if (!hangReady) {
                hangMotor.setTargetPosition(hangTarget);
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (hangReady) {
                hangMotor.setTargetPosition(0);
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }



