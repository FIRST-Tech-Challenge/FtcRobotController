package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DistanceSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.RobotDevices;
import org.firstinspires.ftc.teamcode.threads.TweakableMovementThread;


//threaded tele op controller......
@TeleOp( name="MotorsOnly")
public class ThreadedTeleOpMotorsOnly extends OpMode {

    TweakableMovementThread _move;
    Telemetry.Item _threadCount;//,_bot_cone;
    private BNO055IMU imu         = null;
    RobotDevices robotDevices;

    DistanceSensorDevice bottom_cone;

    @Override
    public void init() {
        telemetry.setAutoClear(false);

        robotDevices =  RobotDevices.getDevices(hardwareMap);
        imu = robotDevices.imu;
        _move = new TweakableMovementThread(gamepad1, robotDevices.wheels, telemetry, imu, 500, false);
        _threadCount = telemetry.addData("Threads", Thread.activeCount());

    }

    @Override
    public void start() {
        _move.start();
    }

    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());

        //_bot_cone.setValue(bottom_cone.getDistanceMM());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        _move.cancel();
    }


}
