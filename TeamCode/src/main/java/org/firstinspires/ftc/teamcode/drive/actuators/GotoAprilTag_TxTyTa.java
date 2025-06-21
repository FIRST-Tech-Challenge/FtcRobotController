package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Goto AprilTag tx ty ta", group = "Autonomous")
public class GotoAprilTag_TxTyTa extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;
    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;

    @Override
    public void runOpMode() {
        // Mapear motores
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

        // Iniciar Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("Pressione PLAY para iniciar");
        telemetry.update();

        lright.setPosition(1);
        lleft.setPosition(0.1);
        rotate.setPosition(0.7);
        pleft.setPosition(0.35);
        pright.setPosition(0.65);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // rotação horizontal
                double ty = result.getTy(); // offset vertical (proxy de distância)
                double ta = result.getTa(); // área do alvo

                // Constantes de controle
                double kTurn = 0.025;
                double kForward = 0.03;

                double turn = tx * kTurn;
                double forward = (15 - ty) * kForward; // ty aumenta conforme se aproxima

                // Limites
                turn = clip(turn, -0.4, 0.4);
                forward = clip(forward, -0.4, 0.4);

                driveMecanum(0, forward, turn);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.update();

                // Parar se estiver alinhado e perto
                if (Math.abs(tx) < 1 && ty > 9 && ta > 1.8) {
                    break;
                }

            } else {
                driveMecanum(0, 0, 0);
                telemetry.addLine("Tag não visível");
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
