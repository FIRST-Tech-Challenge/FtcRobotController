package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Servo + motor Test")
public class ServoForCody extends LinearOpMode {
    private Servo s1, s2;
    private double pos = 0.3333333;

    private static final double L_OPEN_POS = 0.4;
    private static final double L_CLOSED_POS = 0.6;
    private static final double R_OPEN_POS = 0.6;
    private static final double R_CLOSED_POS = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
//        DcMotor m = hardwareMap.dcMotor.get("m");

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Init Done");
        telemetry.update();

        while (opModeIsActive()) {
//            s1.setPosition(0);
//
            if (gamepad1.triangle) {
                s1.setPosition(0);
                s2.setPosition(1);
            } else if (gamepad1.circle) {
                s1.setPosition(0.5);
                s2.setPosition(0.5);
            } else if (gamepad1.cross) {
                s1.setPosition(R_OPEN_POS);
                s2.setPosition(L_OPEN_POS);
            } else if (gamepad1.square) {
                s1.setPosition(R_CLOSED_POS);
                s2.setPosition(L_CLOSED_POS);
            }

//            m.setPower(gamepad1.left_stick_y);

            telemetry.addLine("Servo1 Positions: " + (s1.getPosition() * 180));
            telemetry.addLine("Servo2 Positions: " + (s2.getPosition() * 180));
//            telemetry.addLine("Motor Power: " + m.getPower());
            telemetry.update();
        }
    }
}
