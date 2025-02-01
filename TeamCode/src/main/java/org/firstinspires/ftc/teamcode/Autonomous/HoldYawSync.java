package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;
import java.lang.Double;

public class HoldYawSync {
    IMU imu;
    Input input;
    public HoldYawSync(HardwareMap hardwareMap) {
        imu = new IMU(hardwareMap);
        input = new Input(hardwareMap, true);

    }

    public void setPos(int pos) {
        Double angle = Double.valueOf(imu.getAngle());
        Integer realAngle = angle.intValue();
        boolean tolerance = false;

        while (realAngle != pos && !tolerance) {

            if (Math.abs(realAngle - pos) <= 1) {
                tolerance = true;
            }
            input.spinToPosition(pos);
            angle = Double.valueOf(imu.getAngle());
            realAngle = angle.intValue();

            BotTelemetry.addData("yaw", realAngle);
            BotTelemetry.update();
        }
    }
}
