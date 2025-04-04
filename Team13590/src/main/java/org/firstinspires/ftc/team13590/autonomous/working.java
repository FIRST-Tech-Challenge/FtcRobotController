package org.firstinspires.ftc.team13590.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team13590.PathfinderSoftware;
import org.firstinspires.ftc.team13590.RobotHardware;

@Disabled
@Autonomous(name= "workingSpecimen", group ="LL")
public class working extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        final double MtoIN = 39.3701;
        double x = 0;
        double y = 0;
        double heading;
        Pose3D botPose;


        double BrungX = -6.7; // FIXME find this out
        double BrungY = 29; // FIXME find this out
        double BozX = -35.8; // FIXME find this out
        double BozY = 53.4; // FIXME find this out

        double RrungX = 9.7; // FIXME find this out
        double RrungY = -27.7; // FIXME find this out
        double RozX = 36.2; // FIXME find this out
        double RozY = -54.9; // FIXME find this out

        double actingOzX = 0;
        double actingOzY = 0;
        double actingRungX = 0;
        double actingRungY = 0;

        robot.init(true);
        ptFinder.init(false);


        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(0);
        robot.leftBackDrive.setTargetPosition(0);
        robot.rightFrontDrive.setTargetPosition(0);
        robot.rightBackDrive.setTargetPosition(0);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(0.8);

        limelight.start();
        robot.setClawPosition(robot.enable, robot.superposition, robot.enable);
        waitForStart();



        robot.driveFieldCentric(0.6,0,0);
        robot.encoderFieldCentric(27.5,0,0);
        sleep(100);
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if ((robot.elbowDrive.getCurrentPosition() >= (int) (robot.ARM_COUNTS_PER_DEGREE*100))){
                robot.extensionDrive.setTargetPosition((int) (6.5*robot.COUNTS_PER_MOTOR_REV));
            }
        }
        robot.clawAxial.setPosition(robot.CLAW_MID);

        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }
        while (robot.extensionDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }

        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(70))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
                robot.clawAxial.setPosition(robot.CLAW_MID);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }

        actingOzX = BozX;
        actingOzY = BozY;
        actingRungX = BrungX;
        actingRungY = BrungY;

        sleep(250);

        robot.driveFieldCentric(-0.4,0,0);
        robot.encoderFieldCentric(-5,0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }


        /*
        robot.clawAxial.setPosition(0.7);
        robot.elbowDrive.setTargetPosition(0);
        sleep(3000);

         */
        // return to OZ
        ptFinder.tryAgain(x, y, actingOzX, actingOzY, false);
        robot.encoderFieldCentric((actingOzX - x), (actingOzY - y), 0);
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_COLLAPSED);
        robot.clawPinch.setPosition(robot.CLAW_OPEN);
        robot.clawYaw.setPosition(robot.CLAW_MID);
        robot.clawAxial.setPosition(robot.CLAW_MID);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        while (robot.elbowDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        // begin scoring sequence -> (after picking up all 3 samples)

        sleep(1000);

        telemetry.addData("Done!","");
    }
}