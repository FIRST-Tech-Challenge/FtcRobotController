package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "wobbleGoal")
public class WobbleGoal extends LinearOpMode {

    DcMotorEx wobbleGoalMotor;
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        wobbleGoalMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("wobbleGoalMotor");
        waitForStart();

        wobbleGoalMotor.setPower(0.1);
    }

}

