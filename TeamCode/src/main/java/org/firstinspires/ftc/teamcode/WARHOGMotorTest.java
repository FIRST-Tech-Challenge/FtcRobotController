package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp(name="WARHOGTest", group="")
public class WARHOGMotorTest extends LinearOpMode {

    private DcMotor intake;
    private DcMotor intake1;

    public WARHOGMotorTest() throws InterruptedException {    }

    public void runOpMode() throws InterruptedException{
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");

        waitForStart();

        while(opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
            intake1.setPower(gamepad1.right_stick_y);
        }
    }

}
