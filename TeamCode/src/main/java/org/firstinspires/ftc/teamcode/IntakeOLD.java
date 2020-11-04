package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    //This class will contain the 2 motors that drive the intake system
    //sensors include a touch sensor to stop system when sensor is hit by block (grab block when hit)
    //Methods will include an in/off/reverse

    public CRServo leftIntake  = null;
    public CRServo rightIntake = null;

    HardwareMap hwMap          = null;

    public Intake(){
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftIntake  = hwMap.get(CRServo.class, "left_intake");
        rightIntake = hwMap.get(CRServo.class,"right_intake");
    }

    //Power directions subject to change depending on how servos move when giving positive or negative powers
    public void intakeControl(IntakeDirection direction){
        if (direction == IntakeDirection.IN){
            leftIntake.setPower(1d);
            rightIntake.setPower(1d);
        }if (direction == IntakeDirection.OUT){
            leftIntake.setPower(-1d);
            rightIntake.setPower(-1d);
        }if (direction == IntakeDirection.OFF){
            leftIntake.setPower(0d);
            rightIntake.setPower(0d);
        }
    }
}
*/