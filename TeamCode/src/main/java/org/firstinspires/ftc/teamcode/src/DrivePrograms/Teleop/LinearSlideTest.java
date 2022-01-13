package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "linear_slide", s, this::opModeIsActive, this::isStopRequested);


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

            //intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            if ((gamepad2.left_stick_y) != 0) {
                slide.setMotorPower(gamepad2.left_stick_y);
                slide.setTargetHeight(slide.getEncoderCount());
            } else {
                slide.threadMain();
            }
            telemetry.addData("LS Height: ", slide.getEncoderCount());
            telemetry.addData("Target Height: ", slide.getTargetHeight());
            telemetry.update();

            //Thread.sleep(5000);
            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);

            //Thread.sleep(5000);
            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.Down);

        }
    }
}