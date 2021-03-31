package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Test Drive TeleOp")
public class TestDriveTeleOp extends MasterTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            //setWobbleGoalGrabber();
            moveWobbleGoalArm();
            telemetry.update();

            idle();
        }
    }
}
