package org.firstinspires.ftc.teamcode.visionex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.text.DecimalFormat;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LimelightLocalization extends LinearOpMode {
    private Limelight3A limeLight;
    private int hz = 100;
    private int transInterval = 11;
    private int pipeLine = 0;
    public final double M_TO_IN = 39.3700787402;
    //Different postions of the April tags on the field relative to the origin in inches
    public static final Vector2d[] APRIL_TAG_POSITIONS = new Vector2d[]{
            //11
            new Vector2d(-72,-48),
            //12
            new Vector2d(0,72),
            //13
            new Vector2d(72,-48),
            //14
            new Vector2d(72,48),
            //15
            new Vector2d(0,-72),
            //16
            new Vector2d(-72,48)

    };



    public LimelightLocalization(HardwareMap hw)
    {
        limeLight = hw.get(Limelight3A.class, "limelight");
        limeLight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setPipeLine(int pipeLine)
    {
        this.pipeLine = pipeLine;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //rate of data being sent each second

        while(isStopRequested())
        {
            limeLight.setPollRateHz(hz);
            limeLight.start();
            telemetry.setMsTransmissionInterval(transInterval);
            limeLight.pipelineSwitch(pipeLine);
            limeLight.start();
        }

    }

    //method that gets distance from the AprilTag in inches
    public Pose3D getDistance()
    {
        LLResult result = limeLight.getLatestResult();
        if(result != null && isDataCorrect())
        {
            Pose3D pose = result.getBotpose();
            telemetry.addData("Pose: ", pose);
            return pose;
        }
        telemetry.addData("Distance: ", "No valid data");
        return null;

    }
    //rounds Distance
    public String[] getDistanceInInches()
    {
        DecimalFormat df = new DecimalFormat("#.##");
        Pose3D pose = getDistance();
        String[] distanceInMeters = new String[2];;
        if(pose != null)
        {
            Position pos = pose.getPosition();
            double x = pos.x;
            double y = pos.y;
            double x_offset = 20;
            distanceInMeters[0] = df.format(x *M_TO_IN + x_offset);
            distanceInMeters[1] = df.format(y*M_TO_IN);
            return distanceInMeters;
        }
        distanceInMeters[0] = df.format(0);
        distanceInMeters[1] = df.format(0);
        return distanceInMeters;
    }

    public LLResult getLatestResult() {
        return limeLight.getLatestResult();
    }

    //method that gets check if data is correct
    public boolean isDataCorrect()
    {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

}
