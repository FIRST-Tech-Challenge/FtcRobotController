package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ArmRelease;
import org.firstinspires.ftc.teamcode.hardware.DistanceSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.hardware.RobotDevices;
import org.firstinspires.ftc.teamcode.threads.LiftClawThread;
import org.firstinspires.ftc.teamcode.threads.TweakableMovementThread;


//threaded tele op controller......
public abstract class ThreadedTeleOpBase extends OpMode {

    TweakableMovementThread _move;
    LiftClawThread _liftclaw;
    ArmRelease _armRelease;
    Lights _light;

    Telemetry.Item _threadCount;//,_bot_cone;
    private BNO055IMU imu         = null;
    RobotDevices robotDevices;

    DistanceSensorDevice bottom_cone;

    @Override
    public void init() {
        telemetry.setAutoClear(false);

        robotDevices =  RobotDevices.getDevices(hardwareMap);
        // set up MovementThread
        /*
        final DcMotor [] motors = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};
         */

        //bottom_cone = robotDevices.bottom_cone;
        imu = robotDevices.imu;

        _move = new TweakableMovementThread(gamepad1, robotDevices.wheels, telemetry, imu, 500, false);

        _light = getLights();

        _liftclaw = new LiftClawThread(
                robotDevices.lift_motor,
                robotDevices.lift_servos,
                robotDevices.bottom_stop,
                robotDevices.post_sensor,
                telemetry,
                gamepad2,
                _light
                );

        _threadCount = telemetry.addData("Threads", Thread.activeCount());
        //_bot_cone = telemetry.addData("Bottom_cone", bottom_cone.getDistanceMM());

        _armRelease =  new ArmRelease(hardwareMap.servo.get("ARM_RELEASE"));

    }

    //public abstract void lightOn();


    public abstract Lights getLights();

    @Override
    public void start() {
        _liftclaw.start();
        _move.start();
    }

    @Override
    public void loop() {
        _light.on();
        _threadCount.setValue(Thread.activeCount());

        //_bot_cone.setValue(bottom_cone.getDistanceMM());
        telemetry.update();
        if (gamepad1.left_bumper) {
            _armRelease.set();
        }
        if (gamepad1.right_bumper) {
            _armRelease.release();
        }
    }

    @Override
    public void stop() {
        super.stop();
        _light.off();
        _move.cancel();
        _liftclaw.cancel();
    }


}
