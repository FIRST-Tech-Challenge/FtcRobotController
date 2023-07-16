package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Controller;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class Gripper extends HWMap {
    private  Servo gripper;

    public double position;
    public double _degrees;
    private boolean canGrip = false;
    //

    public Gripper(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }
    public double degreesToServo(double degrees){
        degrees = _degrees/300;

        return degrees;
    }
    public void run(Controller controller){
        if(controller.gamePad1LBumper){
            canGrip = !canGrip;
        }
        if(canGrip){
            position = .75;
            gripper.setPosition(position);
        } else if(canGrip = false){
            position = 1.0;
            gripper.setPosition(position);

        }

        telemetry.addData(">", "gripperPosition: " +  gripper.getPosition());
        telemetry.addData(">", "Position: " +  position);
    }




}