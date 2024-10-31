package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Karine Click Here")
public class karineClickHere extends LinearOpMode {

    private boolean sheepFound;

    private boolean wasX = false;

    public void runOpMode() throws InterruptedException {

        double reducer = 0.7;

        int sheepSoundID = hardwareMap.appContext.getResources().getIdentifier("pig", "raw", hardwareMap.appContext.getPackageName());

        if (sheepSoundID != 0)
            sheepFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, sheepSoundID);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Did Karine find it? (sheep button x)", true);
        telemetry.addData("sheep resource", sheepFound ? "Found" : "Not found /src/main/res/raw" );

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }

            double leftFrontPower = y + x + rx;
            double leftRearPower = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightRearPower = y + x - rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightRearPower));

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            frontLeft.setPower(leftFrontPower*reducer);
            backLeft.setPower(leftRearPower*reducer);
            frontRight.setPower(rightFrontPower*reducer);
            backRight.setPower(rightRearPower*reducer);

            boolean isX = false;
            if (sheepFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, sheepSoundID);
                telemetry.addData("Playing", "Sheep");
                telemetry.update();
            }

            wasX = isX;


        }
    }
}