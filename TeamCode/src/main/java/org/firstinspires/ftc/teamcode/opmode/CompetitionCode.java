package org.firstinspires.ftc.teamcode.opmode;


import android.support.v4.app.INotificationSideChannel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanism.*;

@TeleOp
public class CompetitionCode extends OpMode {

    Traction DriveTrain = new Traction();
// This is the class which does all the computations for each motor's speed.
    ProgrammingBoard Board = new ProgrammingBoard();
// This class directly controls each motor.
    double axial;
    double lateral;
    double yaw;
    double intakePower;
    double armPower;
    boolean alreadyPressed;
    boolean state;


    @Override
    public void init(){
        Board.init(hardwareMap);
// This initializes the hardware map.

        telemetry.addLine("Initialized");
        telemetry.update();

        // By the way, variables defined in here don't extend outside.
    }

    @Override
    public void loop(){

//         The below "if" block is a toggle. It passes the first gate upon the press of A, but then it changes alreadyPressed, which prevents
//         any further changing of state. With the new state, it begins going off the new variables.

        if (gamepad1.a && !alreadyPressed) {
            state = !state;
        }
        alreadyPressed = gamepad1.a;

        if (!state) {
            axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  gamepad1.left_stick_x;
        } else {
            axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  -gamepad1.left_stick_x;
        }

        yaw = gamepad1.right_stick_x;
        intakePower = gamepad1.left_trigger - gamepad1.right_trigger;


        if (gamepad1.right_bumper) {
            armPower = 1;
            telemetry.addLine("Arm up!");
        } else if (gamepad1.left_bumper) {
            armPower = -1;
            telemetry.addLine("Arm down!");
        } else {
            armPower = 0;
            telemetry.addLine("Arm neutral!");
        }

// Set arm and intake power based off variables defined above.
        Board.setIntakePower(intakePower);
        Board.setArmPower(armPower);
        Board.setClawServo(1);
        // Servo supposedly measured where 1 unit = 180 degrees

// This function sends the game pad inputs to the Traction class.
        DriveTrain.controllerDrive(axial, lateral, yaw);

    }
    }