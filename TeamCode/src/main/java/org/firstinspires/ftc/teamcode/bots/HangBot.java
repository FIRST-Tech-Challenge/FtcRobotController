package org.firstinspires.ftc.teamcode.bots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class HangBot extends AutomationBot {
    private DcMotorEx hangMotor1 = null;
    private DcMotorEx hangMotor2 = null;

    public static int hangTarget = 830;
    public static double hangPower = 1.0;
    private boolean hangReady = false;

    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);
        hangMotor1 = hwMap.get(DcMotorEx.class, "hangMotor1");
        hangMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor1.setPower(0);
        hangMotor1.setTargetPosition(0);
        hangMotor2 = hwMap.get(DcMotorEx.class, "hangMotor2");
        hangMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        hangMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor2.setPower(0);
        hangMotor2.setTargetPosition(0);


    }

    public void hang(boolean button) {
        if (button) {
            if (hangMotor1.getCurrentPosition() < hangTarget - 250) {
                hangReady = true;
                hangMotor2.setPower(hangPower);
                hangMotor1.setPower(hangPower);
            } else {
                hangReady = false;
                hangMotor2.setPower(hangPower);
                hangMotor1.setPower(hangPower);
            }
            if (!hangReady) {
                hangMotor1.setTargetPosition(0);
                hangMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor2.setTargetPosition(0);
                hangMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }

        if (hangReady) {
                hangMotor1.setTargetPosition(hangTarget);
                hangMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor2.setTargetPosition(hangTarget);
                hangMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public HangBot(LinearOpMode opMode) {
        super(opMode);
    }

    protected void onTick() {
        super.onTick();

    }

    public String getCurrentPositionString(){
        return "m1:" + hangMotor1.getCurrentPosition() + "; m2:" + hangMotor2.getCurrentPosition() + "; hang state:" + hangReady;
    }
}




