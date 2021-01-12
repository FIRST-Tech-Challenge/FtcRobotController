package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.StrictMath.abs;

// JH: experimental wheel program here!
// as of right now i've pretty much just written a strafing program in my own coding style while trying to understand it
// if i messed something up... in my defense, my first language is javascript
// i have no idea if this will work but if it does i'll take it

@TeleOp(name="Experimental_Teleop_wheels", group="Teleop")
@Disabled
public class EXP_Teleop_wheels extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor collector = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        collector = hardwareMap.get(DcMotor.class, "collector");

        //left side needs to be reversed because the motors are upside down, forehead
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        collector.setDirection(DcMotor.Direction.FORWARD);

        float lfPower;
        float rfPower;
        float lbPower;
        float rbPower;

        float collectPwr;

        float SFPwr;
        float SSPwr;
        //turnPwr: + clockwise, - counterclockwise
        float turnPwr;

        double deadzone = 0.1;
        double basePwrMult = 0.3;
        double highPwrMult = 0.6;
        double collectorPwrMult = -1;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //reset so it doesn't stack
            lfPower = 0.0f;
            rfPower = 0.0f;
            lbPower = 0.0f;
            rbPower = 0.0f;

            collectPwr = 0.0f;

            //set directional energy according to gamepad1

            SFPwr = gamepad1.left_stick_y;
            SSPwr = gamepad1.left_stick_x;
            turnPwr = gamepad1.right_stick_x;

            //set the power vars according to directional energy

            if(abs(SFPwr) > deadzone){
                lfPower -= SFPwr;
                rfPower -= SFPwr;
                lbPower -= SFPwr;
                rbPower -= SFPwr;
            }

            if(abs(SSPwr) > deadzone){
                lfPower += SSPwr;
                rfPower -= SSPwr;
                lbPower -= SSPwr;
                rbPower += SSPwr;
            }

            if(abs(turnPwr) > deadzone){
                lfPower -= turnPwr;
                rfPower += turnPwr;
                lbPower -= turnPwr;
                rbPower += turnPwr;
            }


            //TODO put collector on gamepad2
            //sticking with gamepad1 for ease of testing at this point
            collectPwr = gamepad1.right_trigger;


            lfPower = Range.clip(lfPower, -1, 1);
            rfPower = Range.clip(rfPower, -1, 1);
            lbPower = Range.clip(lbPower, -1, 1);
            rbPower = Range.clip(rbPower, -1, 1);

            collectPwr = Range.clip(collectPwr, 0, 1);

            if (gamepad1.right_bumper){
                lf.setPower(lfPower * highPwrMult);
                rf.setPower(rfPower * highPwrMult);
                lb.setPower(lbPower * highPwrMult);
                rb.setPower(rbPower * highPwrMult);
            } else{
                lf.setPower(lfPower * basePwrMult);
                rf.setPower(rfPower * basePwrMult);
                lb.setPower(lbPower * basePwrMult);
                rb.setPower(rbPower * basePwrMult);
            }

            collector.setPower(collectPwr * collectorPwrMult);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f), leftback (%.2f), rightback (%.2f), collector (%.2f)", lfPower, rfPower, lbPower ,rbPower, collectPwr);
            telemetry.addData("Power Multipliers", "normal (%.2f), sprint (%.2f), collector (%.2f)", basePwrMult, highPwrMult, collectorPwrMult);
            telemetry.addData("Direction", "fw (%.2f), side (%.2f), turn (%.2f)", SFPwr, SSPwr, turnPwr);
            telemetry.update();
        }
    }
}