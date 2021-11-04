package org.firstinspires.ftc.teamcode.Pchassis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class GyroTest extends OpMode {
    Hardware r = new Hardware();
    @Override
    public void init() {
        r.init(hardwareMap);
        r.imu.getRevIMU().startAccelerationIntegration(new Position(), new Velocity(), 10);
    }

    @Override
    public void loop() {
        telemetry.addData("Accel",r.imu.getRevIMU().getAcceleration().toString());
        telemetry.addData("Velocity",r.imu.getRevIMU().getVelocity().toString());
        telemetry.addData("Position",r.imu.getRevIMU().getPosition().toString());
        telemetry.update();
    }
}
