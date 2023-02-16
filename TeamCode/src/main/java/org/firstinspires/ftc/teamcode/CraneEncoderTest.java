package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class CraneEncoderTest extends LinearOpMode {

    private RevColorSensorV3 color;
    private DcMotor crane;
    private DcMotor spin;

    public void runOpMode() {
        color=hardwareMap.get(RevColorSensorV3.class,"Color");
        crane = hardwareMap.get(DcMotor.class, "Crane");
        spin =hardwareMap.get(DcMotor.class,"Spin");
        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crane.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            double cranepower;
            boolean spinpowerup;
            boolean spinpowerdown;
            spinpowerup = gamepad2.dpad_right;
            spinpowerdown =gamepad2.dpad_left;
            cranepower = gamepad2.right_stick_y;

            if (spinpowerdown){
                spin.setPower(-1);
            }
            if (spinpowerup){
                spin.setPower(1);
            }
            if (!spinpowerdown&&!spinpowerup){
                spin.setPower(0);
            }

            crane.setPower(cranepower);

            telemetry.addData("red",color.red());
            telemetry.addData("blue",color.blue());
            telemetry.addData("green",color.green());



            telemetry.addData("spin encoder",spin.getCurrentPosition());
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}


