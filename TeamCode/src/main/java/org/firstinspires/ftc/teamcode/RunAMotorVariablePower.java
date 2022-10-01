package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class RunAMotorVariablePower extends LinearOpMode {

    // Declare Motors
    private DcMotor myMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        myMotor = hardwareMap.get(DcMotor.class, "myMotor");
        myMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while(opModeIsActive()){
            double myMotorPower;

            double drivePower = -gamepad1.left_stick_y; //instead of neg, set dir REVERSE
            myMotorPower = Range.clip(drivePower, -1.0, 1.0);

            myMotor.setPower(myMotorPower);
            telemetry.addData("myMotor", "Forward/Backwards Power (%.2f)", myMotorPower);
            telemetry.update();
        }
    }
}
