package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Odometry Reading Test", group="tests")
public class OdometryReading extends LinearOpMode{

    //for two odometry wheels setup
    private DcMotor parallelEncoder, perpendicularEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        //"shooter"
        parallelEncoder = hardwareMap.get(DcMotor.class, "shooter");
        //"intake"
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "intake");
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("parallel encoder: ", parallelEncoder.getCurrentPosition());
            telemetry.addData("perpendicular encoder: ", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }


}
