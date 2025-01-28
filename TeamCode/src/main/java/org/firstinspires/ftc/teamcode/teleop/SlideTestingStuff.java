
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Motors huh", group="Linear OpMode")
//@Disabled
public class SlideTestingStuff extends LinearOpMode {

    private DcMotor daMotor = null;
    private DcMotor slide1 = null;
    private DcMotor slide2 = null;
    private final int[] slidePosition = {0};
    private boolean slidesIsUp;


    @Override
    public void runOpMode() {

        daMotor = hardwareMap.get(DcMotorEx.class, "Motor7");
        slide1 = hardwareMap.get(DcMotor.class, "Motor5");
        slide2 = hardwareMap.get(DcMotor.class, "Motor6");
        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//
            daMotor.setPower(gamepad1.left_stick_x);
            if (gamepad1.left_trigger != 0) {

                if (gamepad1.right_trigger > 0) {
                    slidePosition[0] += (int) (20 * gamepad1.right_trigger);
                }
                // Move Slide Down
                if (gamepad1.left_trigger > 0) {
                    slidePosition[0] -= (int) (20 * gamepad1.left_trigger);
                }
                //straight up or down
                else if (gamepad1.right_bumper) {
                    slidePosition[0] = Values.slideMax;
                } else if (gamepad1.left_bumper) {
                    slidePosition[0] = 0;
                }

                // Ensure slides stay within bounds
                if (slidePosition[0] < 0) {
                    slidePosition[0] = 0;
                }

                if (slidePosition[0] > Values.slideMax) {
                    slidePosition[0] = Values.slideMax;
                }
                moveSlides(slidePosition[0], Values.velocity);
            }
            if (gamepad1.circle) {
                if (!slidesIsUp) {
                    slidePosition[0] = Values.slideMax;
                    slidesIsUp = true;
                } else if (slidesIsUp) {
                    slidePosition[0] = 0;
                    slidesIsUp = false;
                }
            }

            telemetry.update();
        }
    }
    private void moveSlides ( int distance, double velocity){
        slide1.setTargetPosition(distance);
        slide2.setTargetPosition(distance);

        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide1.setPower(velocity);
        slide2.setPower(velocity);
    }
}

