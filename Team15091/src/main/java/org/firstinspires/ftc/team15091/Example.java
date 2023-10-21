package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Example", preselectTeleOp = "Gamepad")
public class Example extends AutonomousBase {
    DcMotor armMotor;
    Servo grabberServo;

    Thread armDown = new Thread() {
        public void run() {
            armMotor.setTargetPosition(0);
            armMotor.setPower(0.5);
            grabberServo.setPosition(0);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        grabberServo = hardwareMap.get(Servo.class, "grabber_servo");

        TapeDetector redtape = new TapeDetector(hardwareMap.colorSensor.get("sensor_color"));
        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_rear");
        DistanceDetector wall = new DistanceDetector(sensorDistance,15d, true);

        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        setupAndWait();
        //get ready to pick up
        grabberServo.setPosition(0);
        robotDriver.gyroDrive(0.6d, 40d, 0d, 5d, null);
        //pick up stuff -
        grabberServo.setPosition(1);
        sleep(1000);
        armMotor.setTargetPosition(-300);
        armMotor.setPower(0.3);
        sleep(8000);
        robotDriver.gyroTurn(0.6d, -90d, 5d);
        robotDriver.gyroDrive(0.6d, 40d, -90d, 5d, null);
        robotDriver.gyroTurn(0.6d, -180d, 5d);
        robotDriver.gyroDrive(0.6d, 40d, -180d, 5d, null);
        //release to bucket+
        grabberServo.setPosition(0);
        sleep(1000);

    }
}

