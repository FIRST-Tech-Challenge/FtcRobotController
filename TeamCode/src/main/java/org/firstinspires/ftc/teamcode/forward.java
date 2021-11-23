package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.;

@TeleOp(name="forward", group="Teleop")
//@Disabled
public class forward extends LinearOpMode {

    private DcMotor lf = null;  //left front wheel
    private DcMotor rf = null;  //right front wheel
    private DcMotor lb = null;  //left back wheel
    private DcMotor rb = null;  //right back wheel
    private DcMotor tower1 = null; //arm motor 1
    private DcMotor tower2 = null; //arm motor 2
    private Servo clawservo = null; //clawservo

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


        while (opModeIsActive()) {

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

            if (towerPower <= deadzone) {
                towerPower = 0.0f;
            }
            if (towerPower2 <= deadzone) {
                towerPower2 = 0.0f;
            }

            lf.setPower(lPower);
            rf.setPower(rPower);
            lb.setPower(lPower);
            rb.setPower(rPower);
            tower1.setPower(towerPower - towerPower2);
            tower2.setPower(towerPower - towerPower2);


        }
    }
}