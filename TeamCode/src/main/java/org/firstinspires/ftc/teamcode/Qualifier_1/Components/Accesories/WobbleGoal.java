package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

public class WobbleGoal {

    private DcMotor wobbleGoalMotor;
    private LinearOpMode op;
    private final double counts_per_degree = 4;

    public WobbleGoal(LinearOpMode opMode){
        this.op = opMode;

        wobbleGoalMotor = (DcMotor) opMode.hardwareMap.get("wobbleGoalMotor");
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void clockwise(){

        wobbleGoalMotor.setPower(0.25);
    }

    public void counterClockwise(){

        wobbleGoalMotor.setPower(-0.25);
    }

    public void stop(){

        wobbleGoalMotor.setPower(0);

    }
}

