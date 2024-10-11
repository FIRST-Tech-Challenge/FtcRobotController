package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class drivetrain {
    private DcMotor FrontLM = null;
    private DcMotor FrontRM = null;
    private DcMotor BackLM = null;
    private DcMotor BackRM = null;
    private final double wheelDiameter = 3.75;
    private final double PULSE_PER_REVOLUTION = 537.5;
    private LinearOpMode opmode=null;
    IMU imu;

    public drivetrain() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opmode = opMode;
        hwMap=opMode.hardwareMap;
        imu =hwMap.get(IMU.class, "imu");
        imu.resetYaw();

        FrontLM = hwMap.dcMotor.get("FrontLM");
        FrontRM = hwMap.dcMotor.get("FrontRM");
        BackLM = hwMap.dcMotor.get("BackLM");
        BackRM = hwMap.dcMotor.get("BackRM");

        FrontLM.setDirection(FORWARD);
        FrontRM.setDirection(FORWARD);
        BackLM.setDirection(FORWARD);
        BackRM.setDirection(FORWARD);

        FrontRM.setPower(0);
        FrontLM.setPower(0);
        BackLM.setPower(0);
        BackRM.setPower(0);

    }
    public void forward (double speed){
        FrontRM.setPower(speed);
        FrontLM.setPower(speed);
        BackLM.setPower(speed);
        BackRM.setPower(speed);
    }
    public void backward (double speed){
        FrontRM.setPower(-speed);
        FrontLM.setPower(-speed);
        BackLM.setPower(-speed);
        BackRM.setPower(-speed);
    } public void turnRight (double speed){
        FrontRM.setPower(-speed);
        FrontLM.setPower(speed);
        BackLM.setPower(speed);
        BackRM.setPower(-speed);
    } public void turnLeft (double speed){
        FrontRM.setPower(speed);
        FrontLM.setPower(-speed);
        BackLM.setPower(-speed);
        BackRM.setPower(speed);
    } public void strafeRight (double speed){
        FrontRM.setPower(-speed);
        FrontLM.setPower(-speed);
        BackLM.setPower(speed);
        BackRM.setPower(speed);
    } public void strafeLeft (double speed){
        FrontRM.setPower(speed);
        FrontLM.setPower(speed);
        BackLM.setPower(-speed);
        BackRM.setPower(-speed);
    }
}