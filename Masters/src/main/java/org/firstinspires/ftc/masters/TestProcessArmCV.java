package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Config
@Autonomous(name = "Test Process Arm", group = "test")
public class TestProcessArmCV extends LinearOpMode {

    SampleMecanumDrive drive;

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    public static int armTarget = 275;
    int targetPixels = 90;
    int secondTargetPixel = 190;
    int rot_rect_y;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);

        CV.setPipelineProcessArm();

        int armPosition;
        int counter = 0;

        waitForStart();

        armPosition = drive.armMotor.getCurrentPosition();
        armPIDController.setTarget(armTarget);
        drive.armMotor.setPower(armPIDController.calculateVelocity(armPosition));


        long startTime = new Date().getTime();
        long time = 0;
        int yupImTired = 0;

        while (opModeIsActive()) {
            time = new Date().getTime() - startTime;

            armPosition = drive.armMotor.getCurrentPosition();
            armPIDController.setTarget(armTarget);
            drive.armMotor.setPower(armPIDController.calculateVelocity(armPosition));
            rot_rect_y = (int) CV.sleevePipeline.rot_rect.center.y;


            if ( time > 3000) {
                rot_rect_y = (int) CV.sleevePipeline.rot_rect.center.y;

                int encoderDifference = (int) ((rot_rect_y-targetPixels)*0.65);

                armTarget += encoderDifference;
                yupImTired=encoderDifference;

                counter++;
                telemetry.addData("Value changed", yupImTired);
            }
//
//            if (counter < 2 && time > 8000) {
//                rot_rect_y = (int) CV.sleevePipeline.rot_rect.center.y;
//
//                int encoderDifference = (int) ((rot_rect_y-targetPixels)*0.65);
//                yupImTired=encoderDifference;
//
//                armTarget += encoderDifference;
//
//                counter++;
//              //  telemetry.addData("Value changed", yupImTired);
//            }

            telemetry.addData("Value changed", yupImTired);
            telemetry.addData("rot_rect_y: ", rot_rect_y);
            telemetry.addData("arm encoder", drive.armMotor.getCurrentPosition());
            telemetry.update();



        }

    }

}
