package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
@TeleOp(name = "" +
        "Ouch")
public class ResetEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));
        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}