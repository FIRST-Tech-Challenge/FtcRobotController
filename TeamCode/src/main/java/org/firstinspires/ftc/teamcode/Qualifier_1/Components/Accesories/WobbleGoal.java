package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

public class WobbleGoal {

    DcMotorEx wobbleGoalMotor;
    LinearOpMode op;


    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    final double robot_diameter = Math.sqrt(619.84);
    final double wheel_diameter = 3.93701;
    final double counts_per_motor_goBilda = 383.6;
    final double counts_per_inch = (counts_per_motor_goBilda*wheel_diameter * Math.PI)/54.48;  //2*(counts_per_motor_goBilda / (wheel_diameter * Math.PI))
    final double counts_per_degree = counts_per_inch * robot_diameter * Math.PI / 360;

    public WobbleGoal(DcMotorEx inputWobbleGoalMotor,  LinearOpMode opMode){
        this.op = opMode;

        wobbleGoalMotor = inputWobbleGoalMotor;
        wobbleGoalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void startingPosition() {

        int targetPosition = (int)counts_per_degree*0;
        wobbleGoalMotor.setTargetPosition(targetPosition);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    public void grabbingPosition() {

        int targetPosition = (int)counts_per_degree*16;
        wobbleGoalMotor.setTargetPosition(targetPosition);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    public void liftingPosition() {

        int targetPosition = (int)counts_per_degree*71;
        wobbleGoalMotor.setTargetPosition(targetPosition);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    public void droppingPosition() {




//        double ticksToMove = counts_per_degree * liftheight;
//        int newmotorLift = (int) (motorLift.getCurrentPosition() + ticksToMove + 0.5); //adds .5 for rounding
//        //TODO: Check limits for safety
        wobbleGoalMotor.setTargetPosition(1200);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(1.0);

        while (op.opModeIsActive() && wobbleGoalMotor.isBusy() && wobbleGoalMotor.getVelocity() !=0 ) {
            op.telemetry.addData("Lifting ", wobbleGoalMotor.getCurrentPosition() + " velocity=" + wobbleGoalMotor.getVelocity() + " busy=" + wobbleGoalMotor.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //brake
        wobbleGoalMotor.setPower(0);
        wobbleGoalMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



//        int targetPosition = (int)counts_per_degree*90;
//        wobbleGoalMotor.setTargetPosition(targetPosition);
//        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}

