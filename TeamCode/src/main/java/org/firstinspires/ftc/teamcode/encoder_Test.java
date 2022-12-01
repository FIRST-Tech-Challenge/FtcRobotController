package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class encoder_Test extends LinearOpMode {


    private DcMotor crane;

    public void runOpMode() {
        crane = hardwareMap.get(DcMotor.class, "Crane");

        waitForStart();

        while (opModeIsActive()) {
            double throttle;
            throttle = gamepad1.left_stick_y;

            crane.setPower(throttle);
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}


