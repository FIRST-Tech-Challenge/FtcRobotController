package org.firstinspires.ftc.teamcode;
//import statements
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class hardware {
    //motors
    public DcMotor Right_Bottom;
    public DcMotor Right_Top;
    public DcMotor Left_Bottom;
    public DcMotor Left_Top;
    //vuforia
    public VuforiaLocalizer vuforia;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public hardware() {

    }

    public void init(HardwareMap ahwMap) {


        hwMap = ahwMap;
        Right_Bottom = hwMap.get(DcMotor.class, "Right_Bottom");
        Right_Top = hwMap.get(DcMotor.class, "Right_Top");
        Left_Bottom = hwMap.get(DcMotor.class, "Left_Bottom");
        Left_Top = hwMap.get(DcMotor.class, "Left_Top");

        Left_Top.setDirection(DcMotor.Direction.REVERSE);
        Left_Bottom.setDirection(DcMotor.Direction.REVERSE);

        Right_Bottom.setPower(0);
        Right_Top.setPower(0);
        Left_Bottom.setPower(0);
        Left_Top.setPower(0);


    }

}