package org.firstinspires.ftc.team13590.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team13590.PathfinderSoftware;
import org.firstinspires.ftc.team13590.RobotHardware;

@Autonomous(name= "LLLeft", group ="LL")
public class LLchillDay extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);

    private Limelight3A limelight;

    public void score(){
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PARALLEL + robot.angleConvert(15))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(47.5))){
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(70)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }
        robot.extensionDrive.setTargetPosition(0);
        robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
    }

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

        double actingOzX;
        double actingOzY;
        double actingRungX;
        double actingRungY;

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
        robot.calibrateClaw(robot.ELBOW_PARALLEL);
        robot.clawYaw.setPosition(robot.YAW_MID);
        robot.clawPinch.setPosition(robot.CLAW_CLOSE);
        waitForStart();

        sleep(7000);

        robot.driveFieldCentric(0.6,0,0);
        robot.encoderFieldCentric(21.25,0,0);
        sleep(100);
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            if ((robot.elbowDrive.getCurrentPosition() >= (int) (robot.ELBOW_PARALLEL))){
                robot.extensionDrive.setTargetPosition((int) ((robot.EXTENSION_MAXIMUM_COUNT - robot.EXTENSION_COUNTS_PER_REV*4)));
            }
        }

        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
        }

        score();

        if (limelight.getLatestResult() != null && limelight.getLatestResult().getBotpose() != null) {
            botPose = limelight.getLatestResult().getBotpose();
            x = botPose.getPosition().x * MtoIN;
            y = botPose.getPosition().y * MtoIN;

            if ((x < 0) && (y > 0)) {
                actingOzX = BozX;
                actingOzY = BozY;
                actingRungX = BrungX;
                actingRungY = BrungY;
            } else {
                actingOzX = RozX;
                actingOzY = RozY;
                actingRungX = RrungX;
                actingRungY = RrungY;
            }

        } else {
            actingOzX = BozX;
            actingOzY = BozY;
            actingRungX = BrungX;
            actingRungY = BrungY;
        }
        sleep(500);

        robot.driveFieldCentric(-0.4, 0, 0);
        robot.encoderFieldCentric(-3, 0, 0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            telemetry.addData("Busy", "");
            telemetry.update();
        }

        // drive to pos
        robot.elbowDrive.setTargetPosition((int) (13 * robot.ARM_COUNTS_PER_DEGREE));
        ptFinder.tryAgain(0, 0, (-46.5), (7.65), false);
        robot.encoderFieldCentric(7.65, -46.5, 0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            telemetry.addData("Busy", "");
            telemetry.update();
            robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            robot.clawPinch.setPosition(robot.CLAW_OPEN);
        }

        // angle of claw
        robot.clawYaw.setPosition(robot.YAW_MID);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy() || robot.extensionDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        // begin pick-up/drop sequence --> (3x)
        for (byte i=0; i<2; i++) {
            robot.elbowDrive.setTargetPosition( (0));
            while (robot.elbowDrive.isBusy()) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
                telemetry.addData("waiting on elbow", "");
                telemetry.update();
            }
            // align claw with arm
            robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);

            // close claw / grip sample
            robot.clawPinch.setPosition(robot.CLAW_CLOSE);
            sleep(600);

            // drive back & strafe to next sample (x)

            robot.driveFieldCentric(-0.5, 0.6, 0);
            robot.encoderFieldCentric(-11.25, (i*13.5) - 13.5, 0);
            while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy() || robot.extensionDrive.isBusy()){
                telemetry.addData("Busy","");
                telemetry.update();
            }
            robot.driveFieldCentric(0, 0, 0.4);
            robot.encoderFieldCentric(0,0, -45);
            robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PERPENDICULAR);
            while (robot.elbowDrive.getCurrentPosition() <= (int) robot.ELBOW_PARALLEL) {
                telemetry.addData("waiting on elbow", "");
                telemetry.update();
            }
            robot.extensionDrive.setTargetPosition((int) robot.EXTENSION_MAXIMUM_COUNT);

            // rotate claw
            robot.clawAxial.setPosition(robot.CLAW_MID);
            robot.clawYaw.setPosition(robot.YAW_MID);
            // wait on elbow & extension
            while (robot.elbowDrive.isBusy() || robot.extensionDrive.isBusy()){
                telemetry.addData("waiting on elbow & extension", "");
                telemetry.update();
            }

            // drop sample
            robot.clawPinch.setPosition(robot.CLAW_OPEN);
            sleep(600);

            // bring arm back a bit
            robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR- robot.angleConvert(5)));
            while (robot.elbowDrive.isBusy()){
                telemetry.addData("waiting on elbow", "");
                telemetry.update();
            }
            // retract for center of mass
            robot.extensionDrive.setTargetPosition(0);
            while (robot.extensionDrive.isBusy()){
                telemetry.addData("waiting on extension", "");
                telemetry.update();
            }

            // straighten up
            robot.driveFieldCentric(0, 0, 0.4);
            robot.encoderFieldCentric(0,0, 45);


            // drive forward to meet sample
            robot.driveFieldCentric(0.5,0,0);
            robot.encoderFieldCentric(11.25, (i*-13.5),0);
            sleep(300);

        }
        // repeat ^

        // once done, paste 3rd sample pick-up here...


        robot.driveFieldCentric(1,0.25,0);
        robot.encoderFieldCentric(28, 7,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        robot.clawAxial.setPosition(robot.CLAW_MID);

        robot.elbowDrive.setTargetPosition(0);
        robot.extensionDrive.setTargetPosition(0);
        while (robot.elbowDrive.isBusy()) {
            robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            telemetry.addData("waiting on elbow", "");
            telemetry.update();
        }

    }

}