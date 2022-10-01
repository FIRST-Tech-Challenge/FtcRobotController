package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

public class PacManTurnToPos {
private double heading        = 0;
private double goalHeading    = 0;
private boolean rightTurn  = false;
private Localizer localizer;
private  MecanumDriveBase mecanumDriveBase;
CompassSensor                compass;


    public PacManTurnToPos(Localizer localizer, MecanumDriveBase mecanumDriveBase) {
        this.localizer = localizer;
        this.mecanumDriveBase = mecanumDriveBase;
    }

    public void handlePacMan(Gamepad gamepad) {
        if(gamepad.dpad_down){
            goalHeading = 180;
        }
        if(gamepad.dpad_up){
            goalHeading = 0;
        }
        if(gamepad.dpad_left){
            goalHeading = 270;
        }
        if(gamepad.dpad_right){
            goalHeading = 90;
        }


    }
}
