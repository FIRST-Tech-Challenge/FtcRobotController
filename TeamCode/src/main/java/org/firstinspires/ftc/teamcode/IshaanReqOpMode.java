package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Control")
public class IshaanReqOpMode extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {
        double motor1Power = -gamepad1.left_stick_y;
        double motor2Power = -gamepad1.right_stick_y;

        if (gamepad1.a) {
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setPower(0);
        } else if (gamepad1.b) {
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setPower(0);
        } else if (gamepad1.x) {
            motor1.setPower(1.0);
            motor2.setPower(1.0);
        } else if (gamepad1.y) {
            motor1.setPower(-1.0);
            motor2.setPower(-1.0);
        } else {
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
        }

        telemetry.addData("Motor1 Power", motor1.getPower());
        telemetry.addData("Motor2 Power", motor2.getPower());
        telemetry.addData("Lakshya", "Is Better");
    }
}
