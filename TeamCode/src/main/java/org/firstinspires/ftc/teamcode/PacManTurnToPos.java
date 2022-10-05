package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.SimplePIDControl;

public class PacManTurnToPos {
private double heading     = 0;
private double goalHeading = 0;
private double turnSpeed   = 0;
private Localizer localizer;
private  MecanumDriveBase mecanumDriveBase;
private SimplePIDControl angleControl;
CompassSensor                compass;



    public PacManTurnToPos(Localizer localizer, MecanumDriveBase mecanumDriveBase) {
        this.localizer = localizer;
        this.mecanumDriveBase = mecanumDriveBase;
        this.angleControl = new SimplePIDControl(10, 0, 30);
    }

    public void handlePacMan(Gamepad gamepad) {
        //makes heading easier for me.
        heading =  localizer.heading;
            if(gamepad.dpad_down){
                angleControl.setTargetValue(180);
            }
            if(gamepad.dpad_up){
                angleControl.setTargetValue(0);
            }
            if(gamepad.dpad_left){
                angleControl.setTargetValue(270);
            }
            if(gamepad.dpad_right){
                angleControl.setTargetValue(90);
            }
        mecanumDriveBase.driveMotors(0,angleControl.update(heading), 0,1);
    }
}
