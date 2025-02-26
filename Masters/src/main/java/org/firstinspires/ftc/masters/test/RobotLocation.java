package org.firstinspires.ftc.masters.test;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

@TeleOp(name = "Robot Loaclization")
public class RobotLocation extends LinearOpMode {

    DcMotor leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        Constants.setConstants(FConstants.class, LConstants.class);
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(new Pose(10,45,0));

        Init init = new Init(hardwareMap);
        Intake intake = new Intake(init, telemetry);

        this.leftFrontMotor = init.getLeftFrontMotor();
        this.rightFrontMotor = init.getRightFrontMotor();
        this.leftRearMotor = init.getLeftRearMotor();
        this.rightRearMotor = init.getRightRearMotor();

        pinpoint = init.getPinpoint();

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                intake.extendSlideMax();
                intake.servoToDrop();

            }
            if (gamepad1.b){
                intake.retractSlide();
            }

            if (gamepad1.x){
                intake.servoToNeutral();
            }

            poseUpdater.update();

            telemetry.addData("x", poseUpdater.getPose().getX());
            telemetry.addData("y", poseUpdater.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(poseUpdater.getPose().getHeading()));
            telemetry.update();

        }
    }
}
