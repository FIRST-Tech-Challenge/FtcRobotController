package org.firstinspires.ftc.teamcode.opmode;


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
    boolean state;
    boolean alreadyPressed;

    @Override
    public void init(){
        Board.init(hardwareMap);
// This initializes the hardware map.
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){

// The "if" block is a toggle.
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

// Arm variable is set. Due to binary variable, it only has 2 modes. Eventually implement an encoder system?
        if (gamepad1.right_bumper) {
            armPower = .25;
        } else {
            armPower = 0;
        }

        if (gamepad1.left_bumper) {
            armPower = -.25;
        }

// Normal arm control is dialed down. This allows super-strength.
        if (gamepad1.y) {
            armPower = -.5;
        } else {
            armPower = 0;
        }

// Set arm and intake power based off variables defined above.
        Board.setIntakePower(intakePower);
        Board.setArmPower(armPower);

// This function sends the game pad inputs to the Traction class.
        DriveTrain.controllerDrive(axial, lateral, yaw);

    }
    }