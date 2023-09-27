package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Noodles {
    //we need a sensor to sense how many pixels we are intaking!
    public final DcMotorEx noodleMotor;
    private final OpMode opMode;
    private boolean isIntake;

    public Noodles(OpMode opMode) {
        this.opMode = opMode;
        noodleMotor= opMode.hardwareMap.get(DcMotorEx.class, "noodles motor");
        noodleMotor.setDirection(DcMotorEx.Direction.FORWARD);
        isIntake=false;
    }

    public void Intake(){
        noodleMotor.setPower(0.5);
        isIntake=true;
    }
    public void Stop(){
        isIntake= false;
        noodleMotor.setPower(0);
    }


    public void reverseIntake(){
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(0.5);
        isIntake=false;
        noodleMotor.setPower(0.5);
    }

    public boolean getIntakeState(){
        return isIntake;
    }

}
