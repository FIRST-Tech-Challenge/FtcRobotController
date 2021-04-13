package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.abs;

@TeleOp(name="Final_Teleop", group="Teleop")
//@Disabled
public class Final_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null; //left front wheel
    private DcMotor rf = null; //right front wheel
    private DcMotor lb = null; //left back wheel
    private DcMotor rb = null; //right back wheel
    private CRServo SoN = null; //collector
    private DcMotor spindoctorL = null; //shooter
    private DcMotor spindoctorR = null; //shooter (opposite, inverted)
    private DcMotor factory = null; //gear train system

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        SoN = hardwareMap.get(CRServo.class, "SoN");
        spindoctorL = hardwareMap.get(DcMotor.class, "spinL");
        spindoctorR = hardwareMap.get(DcMotor.class, "spinR");
        factory = hardwareMap.get(DcMotor.class,"factory");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        SoN.setDirection(CRServo.Direction.FORWARD);
        spindoctorL.setDirection(DcMotorSimple.Direction.FORWARD);
        spindoctorR.setDirection(DcMotorSimple.Direction.REVERSE);
        factory.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double lfPower;
            double rfPower;
            double lbPower;
            double rbPower;
            double spinPower; //see spindoctor  //JH: second spin motor uses same variable, inverted when applied
            double factoryPower; //see factory
            double SPower; //see SoN

            double deadzone = 0.2;

            lfPower = 0.0f;
            rfPower = 0.0f;
            lbPower = 0.0f;
            rbPower = 0.0f;

            factoryPower = 0;
            SPower = 0;
            spinPower = 0;


            //left side strafe drive
            if (abs(gamepad1.left_stick_y) < deadzone && abs(gamepad1.left_stick_x) > deadzone) {
                lfPower = -gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_x;
            } else if (abs(gamepad1.left_stick_y) < deadzone && abs(gamepad1.left_stick_x) < deadzone) {
                lfPower = 0.15 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) / (abs(gamepad1.left_stick_y - gamepad1.left_stick_x)));
                lbPower = 0.15 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) / (abs(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            } else if (abs(gamepad1.left_stick_y) > deadzone && abs(gamepad1.left_stick_x) > deadzone) {
                lfPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            } else if (abs(gamepad1.left_stick_y) > deadzone && abs(gamepad1.left_stick_x) < deadzone) {
                lfPower = gamepad1.left_stick_y;
                lbPower = gamepad1.left_stick_y;
            }
            if (abs(gamepad1.left_stick_y) < 0.05 && abs(gamepad1.left_stick_x) < 0.05) {
                lfPower = 0.0f;
                lbPower = 0.0f;
            }

            //right side strafe drive
            if (abs(gamepad1.right_stick_y) < deadzone && abs(gamepad1.right_stick_x) > deadzone) {
                rfPower = gamepad1.right_stick_x;
                rbPower = -gamepad1.right_stick_x;
            } else if (abs(gamepad1.right_stick_y) < deadzone && abs(gamepad1.right_stick_x) < deadzone) {
                rfPower = 0.15 * ((gamepad1.right_stick_y + gamepad1.right_stick_x) / (abs(gamepad1.right_stick_y + gamepad1.right_stick_x)));
                rbPower = 0.15 * ((gamepad1.right_stick_y - gamepad1.right_stick_x) / (abs(gamepad1.right_stick_y - gamepad1.right_stick_x)));
            } else if (abs(gamepad1.right_stick_y) > deadzone && abs(gamepad1.right_stick_x) > deadzone) {
                rfPower = gamepad1.left_stick_y + gamepad1.right_stick_x;
                rbPower = gamepad1.left_stick_y - gamepad1.right_stick_x;
            } else if (abs(gamepad1.right_stick_y) > deadzone && abs(gamepad1.right_stick_x) < deadzone) {
                rfPower = gamepad1.right_stick_y;
                rbPower = gamepad1.right_stick_y;
            }
            if (abs(gamepad1.right_stick_y) < 0.05 && abs(gamepad1.right_stick_x) < 0.05) {
                rfPower = 0.0f;
                rbPower = 0.0f;
            }

            // JH: and here we have a dpad strafing control system nobody asked for but may be useful?

            if(gamepad1.dpad_up){
                lfPower = 1.0f;
                rfPower = 1.0f;
                lbPower = 1.0f;
                rbPower = 1.0f;
            }
            if(gamepad1.dpad_down){
                lfPower = -1.0f;
                rfPower = -1.0f;
                lbPower = -1.0f;
                rbPower = -1.0f;
            }
            if(gamepad1.dpad_left){
                lfPower = -1.0f;
                rfPower = 1.0f;
                lbPower = -1.0f;
                rbPower = 1.0f;
            }
            if(gamepad1.dpad_right){
                lfPower = 1.0f;
                rfPower = -1.0f;
                lbPower = 1.0f;
                rbPower = -1.0f;
            }


            // wheel power set
            if (gamepad1.right_bumper) {
                // right bumper, "sprint mode"
                lf.setPower(lfPower * 0.5);
                rf.setPower(rfPower * 0.5);
                lb.setPower(lbPower * 0.5);
                rb.setPower(rbPower * 0.5);
            } else {
                // normal power
                lf.setPower(lfPower * 0.25);
                rf.setPower(rfPower * 0.25);
                lb.setPower(lbPower * 0.25);
                rb.setPower(rbPower * 0.25);
            }


            //shooter power set
            if (gamepad2.right_trigger >= deadzone) {
                spinPower = 1;
            } else if (gamepad2.left_trigger >= deadzone) {
                spinPower = -1;
            }
            // JH: no else condition needed, they're set to 0 at the beginning of the cycle

            //collector power set
            if (gamepad2.a) {
                SPower = 0.75;
            } else if (gamepad2.b) {
                SPower = -0.75;
            }

            //gear train power set
            if (gamepad2.x) {
                factoryPower = -0.4;
            } else if (gamepad2.y) {
                factoryPower = 0.4;
            }

            SoN.setPower(SPower);
            spindoctorL.setPower(spinPower);
            spindoctorR.setPower(spinPower); // JH: the motor direction is reverse, so the power shouldn't need to be
            factory.setPower(factoryPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f), rightback (%.2f)", lfPower, rfPower, lbPower, rbPower);
            telemetry.addData("Servos", "power (%.2f)", SPower);
            telemetry.update();
        }
    }
}