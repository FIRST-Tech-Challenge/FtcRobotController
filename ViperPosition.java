package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ViperPosition-Patrick")
public class ViperPosition extends LinearOpMode {

    final double speedControl = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Slides = hardwareMap.dcMotor.get("slides");
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();

        double leftstick = gamepad1.left_stick_y;
        while (opModeIsActive()) {
            leftstick =  gamepad1.left_stick_y;
            leftstick = Math.min(leftstick, 0.7);
            leftstick = Math.max(leftstick, -0.7);

            // set position and limit it
            if (Slides.getCurrentPosition() > -3000 && leftstick < 0){
                //if there is still space to go up
                Slides.setPower(leftstick);
                telemetry.addData("Still space to go up", Slides.getCurrentPosition());
                telemetry.update();
            } else if (Slides.getCurrentPosition() < 0 && leftstick > 0){
                //if there is still space to go down
                telemetry.addData("Still space to go down", Slides.getCurrentPosition());
                telemetry.update();
                Slides.setPower(leftstick);
            } else if (Slides.getCurrentPosition() < -3000 && leftstick < 0) {
                Slides.setPower(0);
                telemetry.addData("no space to go up", Slides.getCurrentPosition());
                telemetry.update();
            } else if (Slides.getCurrentPosition() > -2 && leftstick > 0) {
                Slides.setPower(0);
                telemetry.addData("no space to go down", Slides.getCurrentPosition());
                telemetry.update();
            }
            else {
                Slides.setPower(0);
                telemetry.addData("else", Slides.getCurrentPosition() );
                telemetry.update();
            }

            telemetry.addData("Current position", Slides.getCurrentPosition());
            telemetry.addData("Stick position", leftstick);
            telemetry.update();
        }
    }
}

