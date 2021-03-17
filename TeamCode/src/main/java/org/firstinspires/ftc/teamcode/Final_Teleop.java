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
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private CRServo SoN = null;
    private DcMotor spindoctor = null;
    private DcMotor factory = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        SoN = hardwareMap.get(CRServo.class, "SoN");
        spindoctor = hardwareMap.get(DcMotor.class, "spin");
        factory = hardwareMap.get(DcMotor.class,"factory");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        SoN.setDirection(CRServo.Direction.FORWARD);
        spindoctor.setDirection(DcMotorSimple.Direction.FORWARD);
        factory.setDirection(DcMotor.Direction.FORWARD);


        double SPower = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double lfPower;
            double rfPower;
            double lbPower;
            double rbPower;
            double spinPower;
            double factoryPower;

            lfPower = 0.0f;
            rfPower = 0.0f;
            lbPower = 0.0f;
            rbPower = 0.0f;
            factoryPower = 0.0f;


            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) > 0.2) {
                lfPower = -gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_x;
            } else if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) < 0.2) {
                lfPower = 0.15 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) / (abs(gamepad1.left_stick_y - gamepad1.left_stick_x)));
                lbPower = 0.15 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) / (abs(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            } else if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) > 0.2) {
                lfPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            } else if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) < 0.2) {
                lfPower = gamepad1.left_stick_y;
                lbPower = gamepad1.left_stick_y;
            }

            if (abs(gamepad1.left_stick_y) < 0.05 && abs(gamepad1.left_stick_x) < 0.05) {
                lfPower = 0.0f;
                lbPower = 0.0f;
            }

            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) > 0.2) {
                rfPower = gamepad1.right_stick_x;
                rbPower = -gamepad1.right_stick_x;
            } else if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) < 0.2) {
                rfPower = 0.15 * ((gamepad1.right_stick_y + gamepad1.right_stick_x) / (abs(gamepad1.right_stick_y + gamepad1.right_stick_x)));
                rbPower = 0.15 * ((gamepad1.right_stick_y - gamepad1.right_stick_x) / (abs(gamepad1.right_stick_y - gamepad1.right_stick_x)));
            } else if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) > 0.2) {
                rfPower = gamepad1.left_stick_y + gamepad1.right_stick_x;
                rbPower = gamepad1.left_stick_y - gamepad1.right_stick_x;
            } else if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) < 0.2) {
                rfPower = gamepad1.right_stick_y;
                rbPower = gamepad1.right_stick_y;
            }
            if (abs(gamepad1.right_stick_y) < 0.05 && abs(gamepad1.right_stick_x) < 0.05) {
                rfPower = 0.0f;
                rbPower = 0.0f;
            }

            if (gamepad1.right_bumper) {
                lf.setPower(lfPower * 0.5);
                rf.setPower(rfPower * 0.5);
                lb.setPower(lbPower * 0.5);
                rb.setPower(rbPower * 0.5);
            } else {
                lf.setPower(lfPower * 0.25);
                rf.setPower(rfPower * 0.25);
                lb.setPower(lbPower * 0.25);
                rb.setPower(rbPower * 0.25);
            }

            if (gamepad2.right_trigger >= 0.2) {
                spinPower = 1;
            } else if (gamepad2.left_trigger >= 0.2) {
                spinPower = -1;
            } else {
                spinPower = 0;
            }
            if (gamepad2.a) {
                SPower = -1;
            } else if (gamepad2.b) {
                SPower = 1;
            } else {
                SPower = 0;
            }
            if (gamepad2.x) {
                factoryPower = -0.25;
            } else if (gamepad2.y) {
                factoryPower = 0.25;
            } else {
                factoryPower = 0;
            }

            SoN.setPower(SPower);
            spindoctor.setPower(spinPower);
            factory.setPower(factoryPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f), rightback (%.2f)", lfPower, rfPower, lbPower, rbPower);
            telemetry.addData("Servos", "power (%.2f)", SPower);
            telemetry.update();
        }
    }
}