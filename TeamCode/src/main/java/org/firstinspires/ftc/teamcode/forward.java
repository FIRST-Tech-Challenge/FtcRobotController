package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="forward", group="Teleop")
//@Disabled
public class forward extends LinearOpMode {

    private DcMotor lf = null;  //left front wheel
    private DcMotor rf = null;  //right front wheel
    private DcMotor lb = null;  //left back wheel
    private DcMotor rb = null;  //right back wheel
    private DcMotor tower = null; //arm
    private Servo clawservo = null; //clawservo

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        //tower = hardwareMap.get(DcMotor.class, "tower");
        clawservo = hardwareMap.get(Servo.class,"clawservo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        tower.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double lPower;
            double rPower;
            double towerPower;

            lPower = 0.0f;
            rPower = 0.0f;
            towerPower = 0.0f;

            lPower = gamepad1.left_stick_y;
            rPower = gamepad1.right_stick_y;
            towerPower = gamepad1.right_trigger;

            if (gamepad1.a) {
                clawservo.setPosition(0.0);

            }
            if (gamepad1.b) {
                clawservo.setPosition(1.0);

            }


            lf.setPower(lPower);
            rf.setPower(rPower);
            lb.setPower(lPower);
            rb.setPower(rPower);
            tower.setPower(towerPower);

        }
    }
}
