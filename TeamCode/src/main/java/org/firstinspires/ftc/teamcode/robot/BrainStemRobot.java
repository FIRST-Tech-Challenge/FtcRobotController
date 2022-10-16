package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class BrainStemRobot {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // public DcMotorEx turret;

    //public DcMotorEx extention;



    /*
    public I2cDeviceSynch pixyCam;
    public DigitalChannel expanLimit;
    public DigitalChannel cLimit;
    public DigitalChannel frontLimit;
    public CRServo swod;
    public Servo linearActuator;
    public Servo cServo;
    public Servo cap;
    public Object drive;
*/


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;
    public Turret turret;
    public Lift lift;

    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret  = new Turret(hwMap, telemetry);
        lift    = new Lift(hwMap, telemetry);

        // BTBRT: We do not need to detail the drive train motors here. We will use sampleMecanumDrive in respective entry points (@Autonomous or @Teleop)
        // setMotorModes(currentDrivetrainMode);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

/************************************************************************************************
    // BTBRT: We do not need the methods used for motor modes/powers. sampleMecanumDrive handles it for us.

    private void setMotorModes(DcMotor.RunMode mode) {
        if (mode != currentDrivetrainMode) {
            frontLeft.setMode(currentDrivetrainMode);
            frontRight.setMode(currentDrivetrainMode);
            backLeft.setMode(currentDrivetrainMode);
            backRight.setMode(currentDrivetrainMode);
            currentDrivetrainMode = mode;
        }
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

**********************************************************************************************/

    public void initializeRobotPosition(){
        lift.initializePosition();
        lift.getToClear();
        turret.initializePosition();
        lift.setLiftHeight(0);


    }
    public void moveTurret(int targetDegrees){
        if(!lift.isClear()){
            lift.getToClear();
        }
        turret.moveTurret(targetDegrees);
    }




}
