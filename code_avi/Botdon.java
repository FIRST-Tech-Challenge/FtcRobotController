package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOp Drive", group="TeleOp")
public class Botdon extends OpMode {

    //Variables
    BasicOpTrain dt;
    LinearSlide elevator;


    float slowdownModifier;
    float forwardDrive;
    float panDrive;
    float rotation;
    float elevatorSpeed;


    @Override
    public void init() {
        //Telemetry A
        telemetry.addData("Hello! Initializing!", "＼(⌒▽⌒)");
        telemetry.update();

        //Get Motors
        DcMotor front_left_drive = hardwareMap.dcMotor.get("front left drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front right drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back left drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back right drive");

        // DcMotor elevator_motor = hardwareMap.dcMotor.get("elevator");

        //Init Code
        this.dt = new BasicOpTrain(front_left_drive, front_right_drive, back_left_drive, back_right_drive);
        // this.elevator = new LinearSlide(elevator_motor, 0,360);

        //Telemetry B
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Get Gamepad Vars

        this.slowdownModifier = 1 - (gamepad1.right_trigger * 0.85f);

        this.forwardDrive = - gamepad1.right_stick_y * this.slowdownModifier;
        this.panDrive = gamepad1.right_stick_x * this.slowdownModifier;
        this.rotation = gamepad1.left_stick_x * this.slowdownModifier;

        // this.elevatorSpeed = gamepad2.right_stick_y * this.slowdownModifier;

        //Manage Sriving

        dt.travel(this.forwardDrive, this.panDrive, this.rotation);

        //Manage Linear Slide

        // elevator.MoveSlideUnrestricted(this.elevatorSpeed);

        //Telemetry

        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
    }
}
