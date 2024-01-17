package org.firstinspires.ftc.teamcode.TeleOps.Memdev;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class TemplateTeleOp extends LinearOpMode {
    private DcMotor motor;
    private DcMotor[] motors;
    private Servo servo;
    private CRServo crServo;
    private IMU imu;
    Template template;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g1 = new Gamepad();

        motor = hardwareMap.dcMotor.get("Motor");
        motors = new DcMotor[] {
                hardwareMap.dcMotor.get("Motor 0"),
                hardwareMap.dcMotor.get("Motor 1"),
                hardwareMap.dcMotor.get("Motor 2"),
                hardwareMap.dcMotor.get("Motor 3")
        };
        servo = hardwareMap.servo.get("Servo");
        crServo = hardwareMap.crservo.get("CRServo");
        imu = hardwareMap.get(IMU.class, "imu");

        template = new Template(motor, motors, servo, crServo, imu);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            template.something(g1.left_stick_y);
        }

    }
}
