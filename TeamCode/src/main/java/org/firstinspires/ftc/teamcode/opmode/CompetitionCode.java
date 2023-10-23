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

    @Override
    public void init(){
        Board.init(hardwareMap);
        // This initializes the hardware map.
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral =  gamepad1.left_stick_x;
        yaw     =  gamepad1.right_stick_x;

        DriveTrain.controllerDrive(axial, lateral, yaw);
        // This function sends the game pad inputs to the Traction class.


    }
}
