package org.firstinspires.ftc.teamcode.TestCases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Back Wheels Test", group = "Test")
public class BackWheelsTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addLine("Back wheels found. Press PLAY to spin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            telemetry.addData("backLeft power", backLeft.getPower());
            telemetry.addData("backRight power", backRight.getPower());
            telemetry.update();
        }

        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
//make a edge case that polarity is reversed on the wires make it still go forward fix this in the code make a class that ust moves the wheels forward