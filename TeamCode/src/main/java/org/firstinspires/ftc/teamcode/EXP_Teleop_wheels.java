package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

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

    private DcMotor tower1 = null;
    private DcMotor tower2 = null;

    private Servo clawservo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        tower1 = hardwareMap.get(DcMotor.class, "tower1");
        tower2 = hardwareMap.get(DcMotor.class, "tower2");
        clawservo = hardwareMap.get(Servo.class,"clawservo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        tower1.setDirection(DcMotor.Direction.FORWARD);
        tower2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double lPower;
        double rPower;
        double towerPower;
        double deadzone;
        double towerPower2;

        lPower = 0.0f;
        rPower = 0.0f;
        towerPower = 0.0f;
        deadzone = 0.2f;
        towerPower2 = 0.0f;

        while (opModeIsActive()) {



            lPower = gamepad1.left_stick_y;
            rPower = gamepad1.right_stick_y;
            towerPower = gamepad2.right_trigger;
            towerPower2 = gamepad2.left_trigger;


            if (gamepad2.a) {
                clawservo.setPosition(0.0);
            }
            if (gamepad2.b) {
                clawservo.setPosition(.75);
            }

            if (towerPower <= deadzone){
                towerPower = 0.0f;
            }
            if (towerPower2 <= deadzone){
                towerPower2 = 0.0f;
            }

            lf.setPower(lPower);
            rf.setPower(rPower);
            lb.setPower(lPower);
            rb.setPower(rPower);
            tower1.setPower(towerPower-towerPower2);
            tower2.setPower(towerPower-towerPower2);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f), leftback (%.2f), rightback (%.2f), SoN (%.2f)", lPower, rPower, lPower ,rPower, towerPower-towerPower2);
            //telemetry.addData("Power Multipliers", "normal (%.2f), sprint (%.2f), SoN (%.2f)", basePwrMult, highPwrMult, SoNPwrMult);
            //telemetry.addData("Direction", "fw (%.2f), side (%.2f), turn (%.2f)", SFPwr, SSPwr, turnPwr);
            telemetry.update();
        }
    }
}