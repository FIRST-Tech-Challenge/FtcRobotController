package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@Autonomous(name = "TestMoveAutocorrect2", group = "Concept")
@Disabled
public class MoveAutocorrectTest extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, armMotorLeft, armMotorRight;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    public VoltageReader voltageReader;
    public MoveAutocorrect2 AutoCorrection;
    public RotationDetector rotationDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);
        waitForStart();
        Autocorrect();

    }
    int InitialAngle = 0;
    long Time;
    private void Autocorrect(){
        AutoCorrection.GivenAngle(InitialAngle);
        Time = 150;
        //Time = voltageReader.GetWaitTime(Distance, 1);
        while(Time > 0){
            move.MoveFull(1);
            AutoCorrection.MoveFull(1);
            Time -=1;
            telemetry.addData("Timpul ",Time);
            telemetry.update();
        }
        move.MoveStop();
//        InitialAngle = 60;
//        AutoCorrection.GivenAngle(InitialAngle);
//        while(rotationDetector.WaitForRotation((InitialAngle))){
//            rotate.RotateRaw(1, rotationDetector.VitezaMotoare(InitialAngle));
//        }
//        rotate.RotateStop();
    }
}
