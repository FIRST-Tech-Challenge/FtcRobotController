package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Arm PID", group="pid")
public class ArmPID extends LinearOpMode {

   ArmPIDController armPIDController;

    DcMotorEx armServo;

    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armServo = hardwareMap.get(DcMotorEx.class, "armServo");
       // armServo.setDirection(DcMotorSimple.Direction.REVERSE);
        armServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPIDController = new ArmPIDController(armServo);
        armServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()){
            armPIDController.setTarget(target);
            armServo.setPower(armPIDController.calculateVelocity());
           // telemetry.addData("power", armPIDController.calculateVelocity());
            telemetry.addData("arm", armServo.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
