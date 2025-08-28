package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ViperslideTestAlex")
public class badviperslidesalex extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Slides = hardwareMap.dcMotor.get("slides");
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();

        double leftstick = gamepad1.left_stick_y;
        while (opModeIsActive()) {
            leftstick = -gamepad1.left_stick_y;
            if((Slides.getCurrentPosition() * -1) > 2910) {
                if((leftstick) > 0) {
                    leftstick = 0;
                }
                if((leftstick) < 0) {

                }
                if((leftstick) == 0) {}
            }
            if((Slides.getCurrentPosition()) == 0) {
                if((leftstick) < 0){}
                if((leftstick) < 0) {
                    leftstick = 0;
                }
                if((leftstick * -1) == 0) {}
            }
            if(leftstick >= 0.2) {
                leftstick = 0.2;
            }
            Slides.setPower(leftstick);
            telemetry.addData("Current position",(Slides.getCurrentPosition() * -1) );
            telemetry.addData("Motorpower", leftstick);
            telemetry.update();
        }
    }

}