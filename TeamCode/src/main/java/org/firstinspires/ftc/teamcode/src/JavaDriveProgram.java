package org.firstinspires.ftc.teamcode.src;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotAttachments.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robotAttachments.OdometryPodServos;
import org.firstinspires.ftc.teamcode.robotAttachments.TeleopDriveTrain;


@TeleOp(name = "2022 Drive Program")
public class JavaDriveProgram extends LinearOpMode {

    private TeleopDriveTrain driveTrain;
    private CarouselSpinner spinner;
    private OdometryPodServos pod;


    public void runOpMode() throws InterruptedException {


        boolean xDepressed = true;
        boolean yDepressed = true;
        boolean aDepressed = true;
        boolean bDepressed = true;

        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right", "front_left", "back_right", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        pod = new OdometryPodServos(hardwareMap,"right_odometry_servo","left_odometry_servo","horizontal_odometry_servo");
        pod.raise();

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        double angle = 0;
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);
            angle = angle + 4;

            //Handles Linear Slide Control
            //slide.setMotorPower(0.75 * gamepad2.left_stick_y);

            //Toggles Attachment A Button for Grabber
           /* {
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
            }*/


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
