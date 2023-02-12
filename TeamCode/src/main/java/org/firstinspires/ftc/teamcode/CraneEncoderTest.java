package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class CraneEncoderTest extends LinearOpMode {


    private DcMotor crane;
    private DcMotor spin;

    public void runOpMode() {
        crane = hardwareMap.get(DcMotor.class, "Crane");
        spin =hardwareMap.get(DcMotor.class,"Spin");
        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //crane.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            double throttle;
            boolean spinpowerup;
            boolean spinpowerdown;
            spinpowerup = gamepad2.dpad_right;
            spinpowerdown =gamepad2.dpad_left;
            throttle = gamepad2.left_stick_y;

            if (spinpowerdown){
                spin.setPower(-1);
            }
            if (spinpowerup){
                spin.setPower(1);
            }
            if (!spinpowerdown&&!spinpowerup){
                spin.setPower(0);
            }

            crane.setPower(throttle);
            telemetry.addData("spin encoder",spin.getCurrentPosition());
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}


