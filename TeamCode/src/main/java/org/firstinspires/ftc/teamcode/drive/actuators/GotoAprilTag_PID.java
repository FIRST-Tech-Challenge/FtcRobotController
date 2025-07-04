package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AprilTag Align", group = "Autonomous")
public class GotoAprilTag_PID extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;
    Servo rotate, garra, pleft, pright, lright, lleft;

    private double previousTx = 0;
    private double integralTx = 0;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "odol");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "odor");
        backRight = hardwareMap.get(DcMotor.class, "odom");

        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                // Controle PID para tx
                double kP = 0.015;
                double kI = 0.0002;
                double kD = 0.002;

                integralTx += tx;
                double derivativeTx = tx - previousTx;
                double turn = kP * tx + kI * integralTx + kD * derivativeTx;
                previousTx = tx;

                double forward = (15 - ty) * 0.04;

                turn = clip(turn, -0.4, 0.4);
                forward = clip(forward, -0.4, 0.4);

                driveMecanum(0, forward, turn);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.update();

                if (Math.abs(tx) < 1.0 && ty > 9 && ta > 1.8) {
                    break;
                }
            } else {
                driveMecanum(0, 0, 0);
                telemetry.addLine("Tag não detectada");
                telemetry.update();
            }
        }

        driveMecanum(0, 0, 0);
        limelight.stop();
    }

    private void driveMecanum(double strafe, double forward, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
