package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

/**
 * A Teleop to test the Linear Slide
 */

@TeleOp(name = "LS Test")
public class LinearSlideTest extends LinearOpMode {

    TeleopDriveTrain driveTrain;
    LinearSlide slide;

    public void runOpMode() throws InterruptedException {
        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        {
            DcMotor s = hardwareMap.dcMotor.get("linear_slide");
            s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            s.setPower(1);
            Thread.sleep(100);
            s.setPower(0);
            Thread.sleep(1000);
            s.close();
        }

        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "linear_slide", s, this::opModeIsActive, this::isStopRequested);

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        slide.resetEncoder();
        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.setPowerFromGamepad(gamepad1);
            //slide.setMotorPower(-gamepad2.left_stick_y);


            /*
            if ((gamepad2.left_stick_y) != 0) {
                slide.setMotorPower(gamepad2.left_stick_y);
                slide.setTargetHeight(slide.getEncoderCount());
            } else {
                slide.threadMain();
            }

             */

            slide.setTargetHeight((int) (slide.getTargetHeight() + (10 * -gamepad2.left_stick_y)));
            slide.threadMain();
            if (slide.getTargetHeight() < 0) {
                slide.setTargetHeight(0);
            }
            if (slide.getTargetHeight() > 720) {
                slide.setTargetHeight(720);
            }

            telemetry.addData("Power", -gamepad2.left_stick_y);
            telemetry.addData("LS Height: ", slide.getEncoderCount());
            telemetry.addData("Target Height: ", slide.getTargetHeight());
            telemetry.addData("Slide", slide);
            telemetry.update();

        }
    }
}