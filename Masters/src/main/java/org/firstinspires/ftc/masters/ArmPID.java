package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class ArmPID extends LinearOpMode {

    PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0;

    public static int target =0;

    private final double ticks_in_degree = 1425/360;

    DcMotorEx armServo;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armServo = hardwareMap.get(DcMotorEx.class, "armServo");

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p, i, d);
            int armPos = armServo.getCurrentPosition();
            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target/ticks_in_degree - 30))*f;

            double power = pid +ff;

            armServo.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
