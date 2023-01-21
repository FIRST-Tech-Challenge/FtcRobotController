package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="arm PID")
public class ArmPID extends LinearOpMode {

    PIDController controller;

    public static double p_arm=0.025, i_arm=0.05, d_arm=0.0001;
    public static double f_arm=0.16;

    public static int target =0;

    private final double ticks_in_degree = 1425/360;

    DcMotorEx armServo;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p_arm, i_arm, d_arm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armServo = hardwareMap.get(DcMotorEx.class, "armServo");

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p_arm, i_arm, d_arm);
            int armPos = armServo.getCurrentPosition();
            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f_arm;

            double power = pid +ff;
            if (target ==0 && armPos<10){
                armServo.setPower(0);
            } else {
                armServo.setPower(power);
            }

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
