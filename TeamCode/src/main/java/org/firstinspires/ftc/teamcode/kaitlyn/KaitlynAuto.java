package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class KaitlynAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        encoderMove(12, 0.1);
    }

    private static final double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;
    private static final double P_CONSTANT = 0.005;

    public void encoderMove(int inches, double power) {
        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        int ticks = fLeft.getCurrentPosition();
        double error = (ticksPerInch * 12) - ticks;

        while (opModeIsActive()) {

            telemetry.addLine(String.valueOf(ticks));
            telemetry.addLine(String.valueOf(ticksPerInch * 12));
            telemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(P_CONSTANT*error);


            ticks = fLeft.getCurrentPosition();
            error = (ticksPerInch * 12) - ticks;

        }

    }

    public void wait(int seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();

        while (true) {

            if (elapsedTime.milliseconds() >= seconds*1000) {
                break;
            }

        }
    }
}

