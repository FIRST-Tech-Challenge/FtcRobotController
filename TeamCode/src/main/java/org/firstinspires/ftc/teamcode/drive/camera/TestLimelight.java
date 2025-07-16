package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.json.JSONObject;

import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

@Autonomous(name="GoToTargetAuto", group="Linear Opmode")
public class TestLimelight extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {

        // Mapeia os motores
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Define direção reversa para os motores certos
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Configura o IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParams);

        waitForStart();

        if (opModeIsActive()) {
            Pose robotPose = getLimelightPose();
            double targetX = -1.0; // metros
            double targetY = -1.5;

            if (robotPose != null) {
                // Calcula deslocamento
                double deltaX = targetX - robotPose.x;
                double deltaY = targetY - robotPose.y;

                // Calcula ângulo e distância
                double targetHeading = Math.atan2(deltaY, deltaX); // radianos
                double distance = Math.hypot(deltaX, deltaY);      // metros

                // Converte para movimento de chassis mecanum
                moveToTarget(targetHeading, distance, 0.3);
            }
        }
    }

    private Pose getLimelightPose() {
        try {
            URL url = new URL("http://192.168.1.2:5800/limelight.json"); // Altere se IP for diferente
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1000);

            InputStream in = conn.getInputStream();
            byte[] buffer = new byte[1024];
            int len = in.read(buffer);
            String json = new String(buffer, 0, len);

            JSONObject obj = new JSONObject(json);
            // "botpose_mt2": [x, y, z, roll, pitch, yaw]
            double x = obj.getJSONArray("botpose_mt2").getDouble(0);
            double y = obj.getJSONArray("botpose_mt2").getDouble(1);

            return new Pose(x, y);
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.toString());
            telemetry.update();
            return null;
        }
    }

    private void moveToTarget(double headingRadians, double distanceMeters, double speed) {
        // Conversão de metros para tempo simples (ajuste conforme teste)
        long duration = (long)(distanceMeters * 1000); // Exemplo: 1m = 1000ms

        double x = Math.cos(headingRadians);
        double y = Math.sin(headingRadians);

        // Vetor de potência mecanum
        double fl = y + x;
        double fr = y - x;
        double bl = y - x;
        double br = y + x;

        // Normaliza se necessário
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        // Aplica velocidade
        frontLeft.setPower(fl * speed);
        frontRight.setPower(fr * speed);
        backLeft.setPower(bl * speed);
        backRight.setPower(br * speed);

        sleep(duration);

        // Para os motores
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Estrutura de pose simples
    class Pose {
        double x, y;
        Pose(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
