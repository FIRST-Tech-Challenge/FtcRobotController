package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MantaRayTeleOp", group = "Iterative Opmode")

public class TestTeleOp extends OpMode {
//Naythan is very bad hehe >:)
    /*
    Todo:
    Overall: connect motors to controller on phone
    Steps:
    1. import the imports
    2. add inheritance
    3. add methods to class
    3.5
    4. create initialization variable
        - set values for variables
    5. loop the initilizable variables
    6. set motor to gamepad1 joystick x


     */

    ElapsedTime runtime = new ElapsedTime();
    DcMotor lmotor = null;
    DcMotor rmotor = null;


    @Override
    public void init() {
//initializes variable
        telemetry.addData("Status", "sad :(");
        lmotor = hardwareMap.get(DcMotor.class, "lmotor");
        rmotor = hardwareMap.get(DcMotor.class, "rmotor");
        lmotor.setDirection(DcMotor.Direction.FORWARD);
        rmotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "lmotorized");
    }

    @Override
    public void init_loop() {
//initializes any moving parts
    }

    @Override
    public void start() {
        runtime.reset();
//starts the initial method
    }

    @Override
    public void loop() {
//runs all code to phone?
        double lpower = gamepad1.left_stick_y;
        double rpower = gamepad1.right_stick_y;
        lmotor.setPower(lpower);
        rmotor.setPower(rpower);
        telemetry.addData("status", "lfinished");
    }

    @Override
    public void stop() {
//shuts off motors
    }
}
