package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DriverOperationMode")
public class DriverOp extends RobotOpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void robotloop() {
        gamePadMoveRobot();
        telemetry.addData("POSITION: ", imu.getPosition());
        telemetry.addData("VELOCITY: ", imu.getVelocity());
        telemetry.addData("ANGULAR VELOCITY: ", imu.getAngularVelocity());
        telemetry.addData("TEMPERATURE: ", imu.getTemperature());
        telemetry.addData("E: ", imu.getQuaternionOrientation());
    }
}
