package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    /*
    My questions: do we use DcMotor or DcMotorEx
    luke answer: use DcMotorEx because it has more functionality

     */

    private DcMotorEx motor_fr;
    private DcMotorEx motor_fl;
    private DcMotorEx motor_br;
    private DcMotorEx motor_bl;
    private IMU imu;
    private Gamepad gp1, gp2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        motor_fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motor_br = hardwareMap.get(DcMotorEx.class, "backRight");
        motor_bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        imu = hardwareMap.get(IMU.class, "IMU?");
        gp1 = this.gamepad1;
        gp2 = this.gamepad2;


        telemetry.addData("teleOp is ","initialized");

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("teleOp is ", "running");
            telemetry.update();



        }
    }
}
