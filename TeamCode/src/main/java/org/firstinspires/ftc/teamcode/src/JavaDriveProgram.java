package org.firstinspires.ftc.teamcode.src;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotAttachments.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robotAttachments.Grabber;
import org.firstinspires.ftc.teamcode.robotAttachments.LinearSlide;
import org.firstinspires.ftc.teamcode.robotAttachments.TeleopDriveTrain;


@TeleOp(name = "2022 Drive Program")
public class JavaDriveProgram extends LinearOpMode {

    private TeleopDriveTrain driveTrain;
    private Grabber grabber;
    private LinearSlide slide;
    private CarouselSpinner spinner;


    public void runOpMode() throws InterruptedException {

        boolean xDepressed = true;
        boolean yDepressed = true;
        boolean aDepressed = true;
        boolean bDepressed = true;

        driveTrain = new TeleopDriveTrain(hardwareMap, "back_left", "back_right", "front_left", "front_right");

        grabber = new Grabber(hardwareMap, "freight grabber");
        grabber.close();

        slide = new LinearSlide(hardwareMap, "linear slide arm");

        spinner = new CarouselSpinner(hardwareMap, "carousel wheel");

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(0.75 * gamepad2.left_stick_y);

            //Toggles Attachment A Button for Grabber
            {
                if (!gamepad2.a) {
                    aDepressed = true;
                }
                if (gamepad2.a && grabber.isOpen() && aDepressed) {
                    aDepressed = false;
                    grabber.close();
                }
                if (gamepad2.a && !grabber.isOpen() && aDepressed) {
                    aDepressed = false;
                    grabber.open();
                }

                //Gamepad 2 B Red Duck
                {
                    if (!gamepad2.x) {
                        bDepressed = true;
                    }
                    if (gamepad2.x && bDepressed) {
                        driveTrain.stopAll();
                        spinner.spinOffRedDuck();
                        bDepressed = false;
                    }
                }

                //Gamepad2 X BlueDuck
                {
                    if (!gamepad2.b) {
                        bDepressed = true;
                    }
                    if (gamepad2.b && bDepressed) {
                        driveTrain.stopAll();
                        spinner.spinOffBlueDuck();
                        bDepressed = false;
                    }
                }

                if (gamepad1.b) {
                    driveTrain.setDrivePowerMult(0.3);
                }
                if (gamepad1.x) {
                    driveTrain.setDrivePowerMult(1);

                }
                if (gamepad1.a) {
                    driveTrain.setDrivePowerMult(0.6);
                }
            }
        }
    }
}