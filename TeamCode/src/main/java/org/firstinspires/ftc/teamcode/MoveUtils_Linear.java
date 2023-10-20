package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveUtils_Linear {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime runtime = null;
    private final DcMotorSimple.Direction REVERSE = DcMotorSimple.Direction.REVERSE;
    private final DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    public MoveUtils_Linear(DcMotor lFD, DcMotor lBD,
                            DcMotor rFD, DcMotor rBD, ElapsedTime rt) {
        leftFrontDrive = lFD;
        leftBackDrive = lBD;
        rightFrontDrive = rFD;
        rightBackDrive = rBD;
        runtime = rt;
    }

    public void moveForward(Integer moveTime) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


    }
}
