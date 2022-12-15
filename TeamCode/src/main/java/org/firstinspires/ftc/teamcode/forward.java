package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.;


@TeleOp(name="forward", group="Teleop")
//@Disabled
public class forward extends LinearOpMode {

        // Drive motor and arm variables
        DcMotor lf = null;
        DcMotor rf = null;
        DcMotor lb = null;
        DcMotor rb = null;

        DcMotor tower1 = null;

        // Servo Variables
        Servo clamp = null;

        double SpinLeft = 0.1;
        double STOP = 0.5;
        double SpinRight = 0.9;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //left front wheel
        DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
        //right front wheel
        DcMotor rf = hardwareMap.get(DcMotor.class, "rf");
        //left back wheel
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb");
        //right back wheel
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb");
        //arm motor 1
        DcMotor tower1 = hardwareMap.get(DcMotor.class, "tower1");
        //Clamp
        clamp = hardwareMap.servo.get("clamp");
        telemetry.addData("JServos", "wobble pos (%.2f), claw pos (%.2f)", clamp.getPosition());

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        tower1.setDirection(DcMotor.Direction.FORWARD);
        clamp.setPosition(1);

        waitForStart();
        gamepad1.rumble(1000);
        gamepad2.rumble(1000);
        while (opModeIsActive()) {


            //clamp (NOTE: 0-right limit, left limit will break it)
            if (gamepad2.a) {
                clamp.setPosition(0.5);
            } else {
                clamp.setPosition(1);
            }
            double lPower;
                double towerPower;

                //powerMult = 0.8;
                double deadzone;
                deadzone = 0.2f;

                lPower = gamepad1.left_stick_y;
                double rPower = gamepad1.right_stick_y;
               towerPower = gamepad2.right_trigger;
              



                lf.setPower(lPower * 0.5);
                rf.setPower(rPower * 0.5);
                lb.setPower(lPower * 0.5);
                rb.setPower(rPower * 0.5);
                tower1.setPower(towerPower * 1.0);


            }
        }

    private void setPosition(double v) {
    }

}
