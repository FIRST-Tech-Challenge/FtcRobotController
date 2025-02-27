package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.camera.Limelight;
import org.json.JSONObject;

@Autonomous(name="Detectar Orientação do Sample", group="Linear Opmode")
public class DetectYellowSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Definir a pipeline correta para detectar o sample amarelo
        Limelight.setPipeline(0);  // Troque 0 pelo número da sua pipeline

        waitForStart();

        while (opModeIsActive()) {
            JSONObject limelightData = Limelight.getLimelightData();
            if (limelightData != null) {
                double tx = limelightData.optDouble("tx", 0.0);
                double ty = limelightData.optDouble("ty", 0.0);
                double ta = limelightData.optDouble("ta", 0.0);
                double thor = limelightData.optDouble("thor", 0.0);
                double tvert = limelightData.optDouble("tvert", 0.0);

                double aspectRatio = thor / tvert;
                String orientation = (aspectRatio > 1.5) ? "Deitado" : (aspectRatio < 0.75) ? "De lado" : "Indefinido";

                telemetry.addData("Pipeline Ativada:", 0); // Confirma qual pipeline foi ativada
                telemetry.addData("Posição X:", tx);
                telemetry.addData("Posição Y:", ty);
                telemetry.addData("Área:", ta);
                telemetry.addData("Largura:", thor);
                telemetry.addData("Altura:", tvert);
                telemetry.addData("Razão de Aspecto:", aspectRatio);
                telemetry.addData("Orientação:", orientation);
                telemetry.update();
            }
        }
    }
}
