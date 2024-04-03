package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Test Intake", group="test")
public class testIntake extends LinearOpMode {

    DcMotor intake;
    Servo angle;


    @Override
    public void runOpMode() throws InterruptedException {

       intake = hardwareMap.get(DcMotor.class, "intake");
       angle = hardwareMap.get(Servo.class, "intakeAngle");

      angle.setPosition(CSCons.intakeBottom);

        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a){
                intake.setPower(CSCons.speed);
            }
            if (gamepad1.b){
                intake.setPower(0);
            }
            if (gamepad1.x){
                intake.setPower(-1);
            }

            angle.setPosition(CSCons.intakeBottom);

            telemetry.update();
        }
    }
}
