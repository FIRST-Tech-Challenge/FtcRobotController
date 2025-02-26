package org.firstinspires.ftc.teamcode.drive.camera;

import org.json.JSONObject;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

public class Limelight {

    private static final String LIMELIGHT_URL = "http://limelight.local:5801/get_data";

    // Função para obter dados da Limelight
    public static JSONObject getLimelightData() {
        try {
            URL url = new URL(LIMELIGHT_URL);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");

            Scanner scanner = new Scanner(conn.getInputStream());
            String response = scanner.useDelimiter("\\A").next();
            scanner.close();

            return new JSONObject(response);
        } catch (Exception e) {
            System.out.println("Erro ao obter dados da Limelight: " + e.getMessage());
            return null;
        }
    }
}
