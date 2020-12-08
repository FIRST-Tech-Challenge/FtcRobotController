package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.abs;

// JH: experimental wheel program here!
// as of right now i've pretty much just written a strafing program in my own coding style while trying to understand it
// if i messed something up... in my defense, my first language is javascript
// i have no idea if this will work but if it does i'll take it

@TeleOp(name="Experimental_Teleop_wheels", group="Teleop")
//@Disabled
public class EXP_Teleop_wheels extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        float lfPower = 0.0f;
        float rfPower = 0.0f;
        float lbPower = 0.0f;
        float rbPower = 0.0f;

        float SFPwr;
        float SSPwr;
        //turnPwr: + clockwise, - counterclockwise
        float turnPwr;

        double deadzone = 0.2;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //set directional energy according to gamepad1 left stick

            SFPwr = gamepad1.left_stick_y;
            SSPwr = gamepad1.left_stick_x;

            turnPwr = gamepad1.right_stick_x;


            //set the power vars according to directional energy

            if(abs(SFPwr) > deadzone){
                lfPower += SFPwr;
                rfPower += SFPwr;
                lbPower += SFPwr;
                rbPower += SFPwr;
            }

            if(abs(SSPwr) > deadzone){
                lfPower += SSPwr;
                rfPower -= SSPwr;
                lbPower -= SSPwr;
                rbPower += SSPwr;
            }

            if(abs(turnPwr) > deadzone){
                lfPower += turnPwr;
                rfPower -= turnPwr;
                lbPower += turnPwr;
                rbPower -= turnPwr;
            }

            if (gamepad1.right_bumper){
                lf.setPower(lfPower *0.5);
                rf.setPower(rfPower *0.5);
                lb.setPower(lbPower *0.5);
                rb.setPower(rbPower *0.5);
            } else{
                lf.setPower(lfPower *0.25);
                rf.setPower(rfPower *0.25);
                lb.setPower(lbPower *0.25);
                rb.setPower(rbPower *0.25);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f), rightback (%.2f)", lfPower, rfPower,lbPower ,rbPower);
            telemetry.addData("Direction", "fw (%.2f), side (%.2f), turn (%.2f)", SFPwr, SSPwr, turnPwr);
            telemetry.update();
        }
    }
}