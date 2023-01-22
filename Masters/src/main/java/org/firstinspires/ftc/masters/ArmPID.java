package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="Arm PID")
public class ArmPID extends LinearOpMode {

   ArmPIDController armPIDController;

    DcMotorEx armServo;

    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armServo = hardwareMap.get(DcMotorEx.class, "armServo");
        armPIDController = new ArmPIDController(armServo);


        waitForStart();
        while (opModeIsActive()){
            armPIDController.setTarget(target);
            armServo.setVelocity(armPIDController.calculateVelocity());
        }
    }
}
