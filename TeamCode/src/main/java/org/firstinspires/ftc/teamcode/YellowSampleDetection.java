package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import org.json.JSONObject;
import org.json.JSONArray;

@Autonomous(name = "Yellow Sample Detection", group = "Autonomous")
public class YellowSampleDetection extends CommonUtil {

    private static final String LIMELIGHT_IP = "172.28.0.1";
    private static final String LIMELIGHT_URL = "http://" + LIMELIGHT_IP + ":5807/results";
    private static final double MIN_AREA = 3.0;
    private static final double MAX_AREA = 30.0;
    private static final double MIN_WH_RATIO = 1.8;
    private static final double MAX_WH_RATIO = 2.8;

    private boolean yellowSampleSensed = false;
    private boolean connected = false;
    private int retryCount = 0;
    private static final int MAX_RETRIES = 3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        connected = testConnection();

        if (connected) {
            initializeLimelight();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (!connected && retryCount < MAX_RETRIES) {
                connected = testConnection();
                retryCount++;
            }

            if (connected) {
                yellowSampleSensed = detectYellowSample();

                telemetry.addData("Yellow Sample Detected", yellowSampleSensed);
                if (yellowSampleSensed) {
                    telemetry.addData("Target Area", getArea());
                    telemetry.addData("W/H Ratio", getWHRatio());
                    s3.setPosition(1);
                }
            } else {
                telemetry.addData("Error", "Limelight not connected");
            }

            telemetry.update();
            sleep(20);
        }
    }

    private JSONObject getLimelightData() {
        try {
            URL url = new URL(LIMELIGHT_URL);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");

            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            StringBuilder response = new StringBuilder();
            String inputLine;

            while ((inputLine = in.readLine()) != null) {
                response.append(inputLine);
            }
            in.close();

            return new JSONObject(response.toString());
        } catch (Exception e) {
            telemetry.addData("Data Error", e.getMessage());
            return null;
        }
    }

    private double getArea() {
        try {
            JSONObject data = getLimelightData();
            return data.getJSONArray("Results").getJSONObject(0)
                    .getJSONArray("Targets").getJSONObject(0)
                    .getDouble("ta");
        } catch (Exception e) {
            return -1;
        }
    }

    private double getWHRatio() {
        try {
            JSONObject data = getLimelightData();
            JSONObject target = data.getJSONArray("Results").getJSONObject(0)
                    .getJSONArray("Targets").getJSONObject(0);
            double width = target.getDouble("tlong");
            double height = target.getDouble("tshort");
            if (height <= 0) return 0;
            return width / height;
        } catch (Exception e) {
            return -1;
        }
    }

    private void sendCommand(String key, int value) {
        try {
            URL url = new URL("http://" + LIMELIGHT_IP + ":5807/settings/" + key + "/" + value);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("PUT");
            conn.getResponseCode();
        } catch (Exception e) {
            telemetry.addData("Command Error", key + ": " + e.getMessage());
        }
    }

    private boolean testConnection() {
        try {
            URL url = new URL(LIMELIGHT_URL);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(1000);
            conn.setRequestMethod("GET");
            return (conn.getResponseCode() == 200);
        } catch (Exception e) {
            telemetry.addData("Connection Error", e.getMessage());
            return false;
        }
    }

    private void initializeLimelight() {
        try {
            sendCommand("pipeline", 0);
            sendCommand("camMode", 0);
            sendCommand("ledMode", 3);
            sendCommand("stream", 1);
            sleep(100);
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
    }

    private boolean detectYellowSample() {
        try {
            JSONObject data = getLimelightData();
            if (data != null && data.has("Results")) {
                JSONArray targets = data.getJSONArray("Results").getJSONObject(0).getJSONArray("Targets");
                if (targets.length() > 0) {
                    JSONObject target = targets.getJSONObject(0);
                    double area = target.getDouble("ta");
                    double width = target.getDouble("tlong");
                    double height = target.getDouble("tshort");
                    double ratio = width / height;

                    return (area >= MIN_AREA && area <= MAX_AREA) &&
                            (ratio >= MIN_WH_RATIO && ratio <= MAX_WH_RATIO);
                }
            }
            return false;
        } catch (Exception e) {
            return false;
        }
    }
}