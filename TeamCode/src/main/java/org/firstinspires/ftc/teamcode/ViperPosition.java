package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ViperslideTestAlex")
public class ViperPosition extends LinearOpMode {

    int position = 0;
    final double speedControl = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Slides = hardwareMap.dcMotor.get("slides");
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();

        double leftstick = gamepad1.left_stick_y;
        while (opModeIsActive()) {

            leftstick = -gamepad1.left_stick_y;
            leftstick = Math.min(leftstick, 0.2);

            // set position and limit it
            position = Slides.getCurrentPosition();
            if (position < 2910 && leftstick>0){
                //if there is still space to go up
                Slides.setPower(leftstick);
            } else if (position > 0 && leftstick <0){
                //if there is still space to go down
                Slides.setPower(-leftstick);
            } else {
                Slides.setPower(0);
            }

            telemetry.addData("Current position", (Slides.getCurrentPosition() * -1));
            telemetry.addData("Stick position", leftstick);
            telemetry.update();
        }
    }
}

