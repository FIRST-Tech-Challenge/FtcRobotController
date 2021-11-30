package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;

@TeleOp(name = "LS Test")
public class LinearSlideTest extends TeleopTemplate {
    DcMotor linearSlide;

    public void runOpMode() {
        this.initAll();

        linearSlide = hardwareMap.dcMotor.get("slide_motor");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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