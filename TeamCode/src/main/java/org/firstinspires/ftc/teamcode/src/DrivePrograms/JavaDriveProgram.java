package org.firstinspires.ftc.teamcode.src.DrivePrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;


@TeleOp(name = "2022 Drive Program")
public class JavaDriveProgram extends LinearOpMode {

    private TeleopDriveTrain driveTrain;
    private CarouselSpinner spinner;
    private OdometryPodServos pod;
    private LinearSlide slide;
    private ContinuousIntake intake;


    public void runOpMode() throws InterruptedException {


        boolean xDepressed = true;
        boolean yDepressed = true;
        boolean aDepressed = true;
        boolean bDepressed = true;

        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right", "front_left", "back_right", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        pod = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        pod.raise();

        slide = new LinearSlide(hardwareMap, "slide_motor");

        intake = new ContinuousIntake(hardwareMap, "intake_motor");


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(0.75 * gamepad2.left_stick_y);
            intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);

            /*
            //Gamepad 2 B Red Duck
            {
                if (!gamepad2.x) {
                    bDepressed = true;
                }
                if (gamepad2.x && bDepressed) {
                    driveTrain.stopAll();
                    spinner.spinOffBlueDuck();
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
                    spinner.spinOffRedDuck();
                    bDepressed = false;
                }
            }8/

             */


            if (gamepad2.x){
                spinner.setPowerBlueDirection();
            }else if (gamepad2.b){
                spinner.setPowerRedDirection();
            }else{
                spinner.stop();
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
