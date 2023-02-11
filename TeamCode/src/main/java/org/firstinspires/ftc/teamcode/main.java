package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "donga")

public class main extends LinearOpMode {
    movement movement = null;




//    movement movement = new movement(hardwareMap.get(DcMotor.class, "Motor1"),hardwareMap.get(DcMotor.class, "Motor2"),hardwareMap.get(DcMotor.class, "Motor3"),hardwareMap.get(DcMotor.class, "Motor4"));

    sensor sensor = new sensor();
    composetelemetry composetelemetry = new composetelemetry();

    ARM ARM = new ARM();
    Grabber Grabber= new Grabber();

    @Override
    public void runOpMode() {

        /**
         * Initialize hardware here.
         */
        DcMotor motor1, motor2, motor3, motor4;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");


        // Put hardware into a global list
        ArrayList<DcMotor> movement_motors = new ArrayList<>();
        movement_motors.add(motor1);
        movement_motors.add(motor2);
        movement_motors.add(motor3);
        movement_motors.add(motor4);
        
        movement = new Movement(movement_motors);
        
        

        //initialize motors


        ARM.Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        ARM.Motor6 = hardwareMap.get(DcMotor.class, "Motor6");

        //initialize sensor
        sensor.imu = hardwareMap.get(BNO055IMU.class, "imu");
        sensor.sensorIntitialize();
        sensor.imu.initialize(sensor.parameters);

        waitForStart();

        while (opModeIsActive()){
            gamepader();
        }
    }
    //gamepad input
    public void gamepader(){
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
            movement.verticalMovement(gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.right_stick_y) < Math.abs(gamepad1.right_stick_x)) {
            movement.horizontalMovement(gamepad1.right_stick_x);
        } else {
            movement.stop();
        }
        telemetry.addData("RightY", gamepad1.right_stick_y);
        telemetry.addData("RightX", gamepad1.right_stick_x);
        telemetry.addData("Motor 1", movement.Motor1);
        telemetry.addData("Motor 2", movement.Motor2);
        telemetry.addData("Motor 3", movement.Motor3);
        telemetry.addData("Motor 4", movement.Motor4);
        movement.telmotor();
        telemetry.update();
    }
}
