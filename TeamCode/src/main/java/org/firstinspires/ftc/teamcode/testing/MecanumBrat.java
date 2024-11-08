package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MecanumBrat extends LinearOpMode {

    DcMotor brat;

    Motor lf,lr,rf,rr;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        brat = hardwareMap.dcMotor.get("motorBrat");
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);

        lf = new Motor(hardwareMap, "leftFront");
        lr = new Motor(hardwareMap, "leftRear");
        rf = new Motor(hardwareMap, "rightFront");
        rr = new Motor(hardwareMap, "rightRear");

        drive = new MecanumDrive(lf, rf, lr, rr);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()){

            if(gamepad1.a) brat.setTargetPosition(10);
            if(gamepad1.x) brat.setTargetPosition(150);
            if(gamepad1.y) brat.setTargetPosition(700);

            double speed = 0.6;

            drive.driveRobotCentric(
                    -gamepad1.left_stick_x*speed,
                    gamepad1.left_stick_y*speed,
                    -gamepad1.right_stick_x*speed
            );

            telemetry.addData("Pos", brat.getCurrentPosition());
            telemetry.update();
        }

    }
}
