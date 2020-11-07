package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HMap {

    // Members of the HardwareMap
    public DcMotor TL = null, TR = null, BL = null, BR = null;
    DcMotor IntakeMotor = null;
    DcMotor LauncherMotor = null;

    // Instantiate them
    com.qualcomm.robotcore.hardware.HardwareMap hwMap =  null;
    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public HMap(){

    }

    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        TL = hwMap.get(DcMotor.class, "TL");
        TR = hwMap.get(DcMotor.class, "TR");
        BL = hwMap.get(DcMotor.class, "BL");
        BR = hwMap.get(DcMotor.class, "BR");

        // Set zero power
        TL.setPower(0.0);
        BL.setPower(0.0);
        TR.setPower(0.0);
        BR.setPower(0.0);

        TR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Encoder Stuff
        resetEncoders();

        runtime.reset();
    }

    public void resetEncoders(){
        TL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
