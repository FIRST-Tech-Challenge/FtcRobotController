package org.firstinspires.ftc.teamcode.src;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.src.DrivePrograms.JavaDriveProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;

//@Disabled
@TeleOp(name="LS Test")
public class LinearSlideTest extends LinearOpMode {

    private TeleopDriveTrain driveTrain;
    private CarouselSpinner spinner;
    private OdometryPodServos pod;
    private LinearSlide slide;
    private ContinuousIntake intake;
    private DcMotor linearSlide;

    public void runOpMode() {
        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right", "front_left", "back_right", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        pod = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        pod.raise();

        linearSlide = hardwareMap.dcMotor.get("slide_motor");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new ContinuousIntake(hardwareMap, "intake_motor");


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);


            //Handles Linear Slide Control
            linearSlide.setPower(0.75 * gamepad2.left_stick_y);
            //intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.addData("LS Height: " , linearSlide.getCurrentPosition());
            telemetry.update();

        }
    }
}