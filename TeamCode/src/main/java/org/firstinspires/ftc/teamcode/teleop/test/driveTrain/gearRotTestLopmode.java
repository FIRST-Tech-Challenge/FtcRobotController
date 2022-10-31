package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.teleop.test.driveTrain.gearRotTest;

@Autonomous(name = "Gear Rot Test", group = "Routes")
//@Disabled
public class gearRotTestLopmode extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    int targetClicks = (int)(90 * constants.CLICKS_PER_DEGREE);
    double power = 0.4;

    public enum DriveType{
        TOP,
        BOTTOM,
        BOTH
    }
    gearRotTest.DriveType dtype = gearRotTest.DriveType.BOTH;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        while (!opModeIsActive()) { //init stage
            UpdateButton();
            if (x.getState() == Button.State.TAP){
                dtype = gearRotTest.DriveType.TOP;
            } else if (y.getState() == Button.State.TAP){
                dtype = gearRotTest.DriveType.BOTTOM;
            } else if (a.getState() == Button.State.TAP){
                dtype = gearRotTest.DriveType.BOTH;
            }
        }

        waitForStart();

        switch(dtype){
            case TOP:
                robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks);
                robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks);

                robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case BOTTOM:
                robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks);
                robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks);

                robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case BOTH:
                robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks);
                robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks);
                robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks);
                robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks);

                robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }


        while (opModeIsActive() && robot.wheelsAreBusy()){
            switch(dtype){
                case TOP:
                    robot.topL.setPower(power);
                    robot.topR.setPower(power);
                    break;

                case BOTTOM:
                    robot.botL.setPower(power);
                    robot.botR.setPower(power);
                    break;

                case BOTH:
                    robot.topL.setPower(power);
                    robot.botL.setPower(power);
                    robot.topR.setPower(power);
                    robot.botR.setPower(power);
                    break;
            }
        }
        robot.setMotorPower(0);
    }
}