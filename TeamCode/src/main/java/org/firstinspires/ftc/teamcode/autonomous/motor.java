package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "motor", group = "Linear OpMode")
public class motor extends LinearOpMode {
    private DcMotor armRotator = null;

    @Override
    public void runOpMode() {
        armRotator = hardwareMap.get(DcMotor.class, "armRotator");

        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            armRotator.setPower(1);

            telemetry.addData("Power", armRotator.getPower());
            telemetry.update();
        }
    }
}
