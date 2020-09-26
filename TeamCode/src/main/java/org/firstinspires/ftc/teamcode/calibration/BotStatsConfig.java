package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.bots.RobotMovementStats;

import java.io.File;
import java.io.Serializable;

public class BotStatsConfig implements Serializable {
    public static String BOT_STATS_CONFIG = "bot-stats.json";

    private RobotMovementStats[] statsForward = new RobotMovementStats[9];
    private RobotMovementStats[] statsBack = new RobotMovementStats[9];

    public File getCalibConfigFile(){
        return AppUtil.getInstance().getSettingsFile(BotStatsConfig.BOT_STATS_CONFIG);
    }
    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static BotStatsConfig deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, BotStatsConfig.class);
    }

    public static BotStatsConfig loadConfig(Telemetry telemetry){
        BotStatsConfig config = new BotStatsConfig();
        try{
            File calibFile = AppUtil.getInstance().getSettingsFile(BotStatsConfig.BOT_STATS_CONFIG);
            if (calibFile.exists()) {
                String data = ReadWriteFile.readFile(calibFile);
                config = BotStatsConfig.deserialize(data);
            }
        }
        catch (Exception ex){
            telemetry.addData("Error Load Stats", ex.getMessage());
            telemetry.update();
        }
        return config;
    }

    public void saveConfig(Telemetry telemetry){
        try{
            File file = getCalibConfigFile();
            ReadWriteFile.writeFile(file, this.serialize());
        }
        catch (Exception ex){
            telemetry.addData("Error Save Stats", ex.getMessage());
            telemetry.update();
        }
    }

    public void setStatsForward(double power, RobotMovementStats data){
       int index =  MotorReductionBot.getPowerIndex(power);
       if (index >= 0){
           statsForward[index] = data;
       }
    }

    public void setStatsBack(double power, RobotMovementStats data){
        int index =  MotorReductionBot.getPowerIndex(power);
        if (index >= 0){
            statsBack[index] = data;
        }
    }
}
