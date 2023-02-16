package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class CraneEncoderTest extends LinearOpMode {

    private Rev2mDistanceSensor distance2;
    private DcMotor crane;
    private DcMotor spin;

    public void runOpMode() {
        distance2=hardwareMap.get(Rev2mDistanceSensor.class,"distance 2");
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


            telemetry.addData("distance2",distance2.getDistance(DistanceUnit.INCH));




            telemetry.addData("spin encoder",spin.getCurrentPosition());
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}


