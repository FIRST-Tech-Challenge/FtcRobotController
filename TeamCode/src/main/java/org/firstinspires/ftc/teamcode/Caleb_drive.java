package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.abs;


@TeleOp(name="Caleb_drive", group="Teleop")
//@Disabled
public class Caleb_drive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor collecter = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        collecter = hardwareMap.get(DcMotor.class, "collecter");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        collecter.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double lfPower;
            double rfPower;
            double lbPower;
            double rbPower;
            double collectorPower;
            double multiply;

            lfPower = 0.0f ;
            rfPower = 0.0f ;
            lbPower = 0.0f ;
            rbPower = 0.0f ;
            multiply = 1/(1-gamepad1.right_trigger)*(1-gamepad1.left_trigger)*0.25;

            collectorPower = gamepad2.left_stick_y;
            collecter.setPower(collectorPower);

            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = -gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_x;
            } else
            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = 0.15 * ((gamepad1.left_stick_y - gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y - gamepad1.left_stick_x)));
                lbPower = 0.15 * ((gamepad1.left_stick_y + gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = gamepad1.left_stick_y - gamepad1.left_stick_x ;
                lbPower = gamepad1.left_stick_y + gamepad1.left_stick_x ;
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = gamepad1.left_stick_y;
                lbPower = gamepad1.left_stick_y;
            }

            if (abs(gamepad1.left_stick_y) < 0.05 && abs(gamepad1.left_stick_x) < 0.05){
                lfPower = 0.0f ;
                lbPower = 0.0f ;
            }

            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.right_stick_x;
                rbPower = -gamepad1.right_stick_x;
            } else
            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = 0.15 * ((gamepad1.right_stick_y + gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y + gamepad1.right_stick_x)));
                rbPower = 0.15 * ((gamepad1.right_stick_y - gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y - gamepad1.right_stick_x)));
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.left_stick_y + gamepad1.right_stick_x ;
                rbPower = gamepad1.left_stick_y - gamepad1.right_stick_x ;
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = gamepad1.right_stick_y;
                rbPower =  gamepad1.right_stick_y;
            }
            if (abs(gamepad1.right_stick_y) < 0.05 && abs(gamepad1.right_stick_x) < 0.05){
                rfPower = 0.0f ;
                rbPower = 0.0f ;
            }


            lf.setPower(lfPower * multiply);
            rf.setPower(rfPower * multiply);
            lb.setPower(lbPower * multiply);
            rb.setPower(rbPower * multiply);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f), rightback (%.2f)", lfPower, rfPower,lbPower ,rbPower);
            telemetry.update();
        }
    }
}