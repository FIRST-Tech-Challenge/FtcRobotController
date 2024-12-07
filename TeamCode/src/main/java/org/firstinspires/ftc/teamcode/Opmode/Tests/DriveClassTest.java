package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Wrist;
import org.firstinspires.ftc.teamcode.Usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;


@TeleOp
@Config
@Disabled
public class DriveClassTest extends LinearOpMode {
    double oldTime = 0;
    double globalStateMachine = 0;
    double timeStamp = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPosition = new Pose2d(0, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drive = new Drivetrain(hardwareMap, startPosition);
        Slides slides = new Slides(hardwareMap, drive.getSlidesMotor());
        Arm arm = new Arm(hardwareMap, drive.getArmMotor());
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        stickyGamepad gp = new stickyGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            if(gp.right_bumper){
                if(globalStateMachine != 9 || globalStateMachine !=6 || globalStateMachine !=3){
                    globalStateMachine++;
                }
            }else if(gp.left_bumper){
                if(globalStateMachine == 3){
                    globalStateMachine = 0;
                }else{
                    globalStateMachine--;
                }

            }

            if(globalStateMachine<0){
                globalStateMachine = 0;
            }
            if(globalStateMachine == 0){ //default position
                arm.preTake();
                slides.floorIntake();
                wrist.intake();
                claw.open();
            }else if(globalStateMachine == 1){
                arm.intake();
            }else if(globalStateMachine == 2){
                claw.close();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            }else if(globalStateMachine == 3){
                if(timeStamp + 250 < timer.milliseconds()){
                    arm.preTake();
                    slides.floorIntake();
                }
            }else if(globalStateMachine == 4){
                arm.deposit();
                slides.preScore();
            }else if(globalStateMachine ==5){
                slides.score();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            }else if(globalStateMachine == 6){
                if(timeStamp+400 < timer.milliseconds()){
                    wrist.deposit();
                }
            }else if(globalStateMachine == 7){
                claw.open();
            }else if(globalStateMachine == 8){
                wrist.intake();
                slides.preScore();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            }else if(globalStateMachine == 9){
                if(timeStamp+450 < timer.milliseconds()){
                    globalStateMachine = 0;
                }
            }

            drive.update();
            drive.setPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("Position", data);
            telemetry.addData("Heading Velocity: ", drive.getHeadingVelocity());
            telemetry.addData("Status", drive.getStatus());
            telemetry.addData("Hub loop Time: ", frequency);
            telemetry.addData("slides inches: ", slides.getCurrentSlidesPosition());
            telemetry.addData("arm degrees:", arm.getCurrentArmPosition());
            slides.update();
            arm.update();
            gp.update();
            telemetry.update();
        }
    }
}
