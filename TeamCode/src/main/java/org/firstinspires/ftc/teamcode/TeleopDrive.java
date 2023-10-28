package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.tools.SetMotors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FC_square_TO_drive", group = "Testing")
public class TeleopDrive extends LinearOpMode {
    private SetMotors setMotorsObj;
    //private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public void Setup(){
        setMotorsObj = new SetMotors(hardwareMap, gamepad1);
    }

    public boolean atRest(){
        return(
                Math.abs(gamepad1.left_stick_y) < setMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.right_stick_y) < setMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.left_stick_x) < setMotorsObj.DEADZONE_MIN_X &&
                        Math.abs(gamepad1.right_stick_x) < setMotorsObj.DEADZONE_MIN_X);

    }

    @Override
    public void runOpMode(){
        Setup();
        waitForStart();
        while(opModeIsActive()){
//            if(drive.isBusy()&& atRest()){
//                drive.update();
//            }
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;
//            if (!drive.isBusy() || !atRest()) {
//                setMotorsObj.driveCommands(hardwareMap, horizontal, vertical, turn, goFast);
//            }
            setMotorsObj.driveCommands(horizontal, vertical, turn, goFast);
        }
    }
}