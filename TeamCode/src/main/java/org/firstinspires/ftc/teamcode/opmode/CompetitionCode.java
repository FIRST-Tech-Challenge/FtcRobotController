package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanism.*;

@TeleOp
public class CompetitionCode extends OpMode {

    Traction DriveTrain = new Traction();
    ProgrammingBoard Board = new ProgrammingBoard();
    double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
    double lateral =  gamepad1.left_stick_x;
    double yaw     =  gamepad1.right_stick_x;

    @Override
    public void init(){
        Board.init(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        DriveTrain.controllerDrive(axial, lateral, yaw);
    }
}
