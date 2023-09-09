package org.firstinspires.ftc.teamcode.robots.r2v2.vision;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.taubot.util.TelemetryProvider;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config
public abstract class VisionProvider implements TelemetryProvider {

    public static int MAX_POSITIONS = 50;

    private List<Position> positions;
    private Position mostFrequentPosition;
    private Bitmap dashboardImage;
    private FtcDashboard dashboard;
    private boolean saveDashboard;
    public Robot robot;

    public VisionProvider() {
        mostFrequentPosition = Position.HOLD;

        positions = new ArrayList<>();

        dashboard = FtcDashboard.getInstance();
        saveDashboard = false;
    }

    abstract public void initializeVision(HardwareMap hardwareMap);

    public void initializeVision(HardwareMap hardwareMap, Robot robot){
        this.robot = robot;
        initializeVision(hardwareMap);
    }

    abstract public void shutdownVision();

    abstract public Position getPosition();

    abstract public void reset();

    abstract public boolean canSendDashboardImage();

    abstract public Bitmap getDashboardImage();

    private void updateMostFrequentPosition() {
        int leftCount = 0, middleCount = 0, rightCount = 0;
        for(int i = 0 ; i < positions.size(); i++) {
            if(i >= MAX_POSITIONS) {
                positions.remove(i);
                i--;
            } else
                switch(positions.get(i)) {
                    case LEFT:
                        leftCount++;
                        break;
                    case MIDDLE:
                        middleCount++;
                        break;
                    case RIGHT:
                        rightCount++;
                        break;
                }
        }

        if(leftCount >= middleCount && leftCount >= rightCount)
            mostFrequentPosition = Position.LEFT;
        else if(middleCount >= leftCount && middleCount >= rightCount)
            mostFrequentPosition = Position.MIDDLE;
        else
            mostFrequentPosition = Position.RIGHT;

    }

    private void sendDashboardImage() {
        dashboardImage = getDashboardImage();
        if(dashboardImage != null) {
            dashboard.sendImage(dashboardImage);
            if (saveDashboard){
                savePreview(dashboardImage);
                saveDashboard=false;
            }
        }
    }

    public void saveDashboardImage() {
        saveDashboard=true; //next call to sendDashboard will try to save the image
    }

    abstract protected void updateVision();

    public void update() {
        updateVision();

        Position position = getPosition();
        if(position != Position.HOLD && position != Position.NONE_FOUND)
        {
            positions.add(0, position);
            updateMostFrequentPosition();
        }
        if(canSendDashboardImage())
            sendDashboardImage();
    }

    public Position getMostFrequentPosition() {
        return mostFrequentPosition;
    }

    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Most Frequent Detected Position", mostFrequentPosition);
        telemetryMap.put("Detected Position", getPosition());

        return telemetryMap;
    }

    public void savePreview(Bitmap bitmap) {
        if (isExternalStorageWritable()) {
            saveImage(bitmap);
        } else {
            //prompt the user or do something
        }
    }

    private void saveImage(Bitmap finalBitmap) {

        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root + "/FIRST/cam_snaps");
        myDir.mkdirs();

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        String fname = "Snap_"+ timeStamp +".png";

        File file = new File(myDir, fname);
        if (file.exists()) file.delete ();
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /* Checks if external storage is available for read and write */
    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }
}
