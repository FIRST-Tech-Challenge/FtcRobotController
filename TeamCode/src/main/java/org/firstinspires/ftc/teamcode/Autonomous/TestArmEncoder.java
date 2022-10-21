package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;

@Autonomous(name = "TestArmEncoder", group="Concept")
@Disabled
public class TestArmEncoder extends LinearOpMode {
    private DcMotorEx armMotorLeft, armMotorRight;
    private ArmEncoder armEncoder;


    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);

        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        sleep(2000);

        while(armEncoder.moveArm(5)){

            int position = armMotorLeft.getCurrentPosition();

            if(position==-75)
            {
                break;
            }

        }

        while(opModeIsActive()){
            telemetry.addData("velocityRight: ", armMotorRight.getVelocity());
            telemetry.addData("velocityLeft: ", armMotorLeft.getVelocity());
            telemetry.addData("positionRight: ", armMotorRight.getCurrentPosition());
            telemetry.addData("positionLeft: ", armMotorLeft.getCurrentPosition());
            telemetry.addData("isAtRight: ", !armMotorRight.isBusy());
            telemetry.addData("isAtLeft: ", !armMotorLeft.isBusy());
            telemetry.update();

        }


    }
}
