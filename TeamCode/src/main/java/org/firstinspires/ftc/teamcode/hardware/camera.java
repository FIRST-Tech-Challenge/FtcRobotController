package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class camera {
    RobotHardwareMap robotHardwareMap;


    LinearOpMode opMode;

    private Limelight3A limelight;
    private int goalColor;// 0 is 20 blue , 1 is 24 red
    private LLResult llResult ;

    // finish the init method

    public camera(RobotHardwareMap robotHardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        limelight = robotHardwareMap.baseHardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // 0 is 20, 1 is 24
        goalColor = 0;
    }

    public void start(){
        limelight.start();
        limelight.pipelineSwitch(2);
    }

    public void loop(){
        YawPitchRollAngles orentation = robotHardwareMap.chImu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orentation.getYaw());
        llResult = limelight.getLatestResult();

        if ((llResult != null) && llResult.isValid()){
            Pose3D botpos = llResult.getBotpose_MT2();
            opMode.telemetry.addLine(String.format("XYA %6.1f %6.1f  %6.3f ", llResult.getTx(),  llResult.getTy(), llResult.getTa()));
            opMode.telemetry.addData("BotPos", botpos.toString());
            opMode.telemetry.addData("Yaw", botpos.getOrientation().getYaw());
            opMode.telemetry.addData("AprilTag Distance (area proxy)", String.format("%.2f%%", llResult.getTa()));
            List<LLResultTypes.FiducialResult> tags = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                opMode.telemetry.addLine("Tag ID: " + tag.getFiducialId());
            }
        }
    }

    public int checkTags(int selection) {
        int minId;
        int maxId;
        limelight.pipelineSwitch(2);
        switch (selection) {
            case 1: minId = 42; maxId = 45; break; // blue scoring
            case 2: minId = 30; maxId = 33; break; // red scoring
            case 3: minId = 38; maxId = 41; break; // blue audience
            case 4: minId = 34; maxId = 37; break; // red audience
            default: return 0;
        }

        Set<Integer> found = new HashSet<>();
        for (int run = 0; run < 40 && !opMode.isStopRequested(); run++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    if (id >= minId && id <= maxId) {
                        found.add(id);
                    }
                }
            }
            opMode.sleep(50); // gives the Limelight time to produce a new frame
        }

        // optional: delete this line if you don't want it on the screen
        opMode.telemetry.addData("Tags found (group " + selection + ")", found.toString());

        return found.size() >= 2 ? 1 : 0;
    }

    public double Robotallignwithgoal(double twistin) {
        double twistout = twistin;
        double powerFactor = 90; // long range value by default

        if ((llResult != null) && llResult.isValid()){
            if ((llResult.getTx() <= 10) && (llResult.getTx() >= -10)) {
                powerFactor = 35;
            }
            twistout= (llResult.getTx() / powerFactor) ;
            //         = twistin;  if you want to conser input from the stick add twist in here
            // value of 180 is rough estimate on the width of the camera at max distance it can pick up april tag(inches)

        }
        return twistout;
    }
    // y is power
    // p is area
    public double outtakespeedfordistance(double powerin) {
        double power = powerin;
        double y2 = 1.0; // april tag area closes point from april tag
        double y1 = 0.5; // april tag area furthest point to april tag
        double p2 = 900;  // power cloest point from april tag
        double p1 = 960; // power  furthest point to april tag
        double m = (p2 - p1) / (y2 - y1);
        double b = (p2 - (m * y2));
        if ((llResult != null) && llResult.isValid()){
            power= (m*llResult.getTa()) + b;
        }

        return power;
    }


    public void goalcolor (int goalColorIn) {
        goalColor = goalColorIn;
        limelight.pipelineSwitch(goalColor); // 0 is 20 blue, 1 is 24 red


    }

}

