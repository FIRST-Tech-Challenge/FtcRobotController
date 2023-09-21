package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Noodles {
    public final DcMotorEx noodleMotor;
    private final OpMode opMode;
    private boolean isIntake;

    public Noodles(OpMode opMode) {
        this.opMode = opMode;
        noodleMotor= opMode.hardwareMap.get(DcMotorEx.class, "left slides motor");
        noodleMotor.setDirection(DcMotorEx.Direction.FORWARD);
        isIntake=false;
    }

    public void Intake(){
        noodleMotor.setPower(0.5);
        isIntake=true;
    }

    public void reverseIntake(){
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(0.5);
        isIntake=false;
    }

}
