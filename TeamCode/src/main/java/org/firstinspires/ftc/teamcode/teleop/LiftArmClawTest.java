package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="LiftArmClawTest", group = "Subsystem Tests")
public class LiftArmClawTest extends LinearOpMode {


    private int index = 0;
    private double[] pidValues;

    private double INCREMENT = 0.01;
    @Override
    public void runOpMode() throws InterruptedException {
        //Init Phase
        Mecanum robot = new Mecanum(hardwareMap);

        Lift lift = new Lift(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        Arm arm = new Arm(hardwareMap);

        Claw claw = new Claw(hardwareMap);
        ActiveIntake intake = new ActiveIntake(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        GamepadEvents controller2 = new GamepadEvents(gamepad2);

//        DcMotor liftL = hardwareMap.get(DcMotor.class,"liftLeft");
//        DcMotor liftR = hardwareMap.get(DcMotor.class,"liftRight");
//        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialized as 6 because both Lift and Arm each have their own PID
        pidValues = new double[8];
        for (double val : lift.getPid().getPIDValues()){
            pidValues[index] = val;
            updateIndex(true);
        }
        for (double val : arm.getPid().getPIDValues()){
            pidValues[index] = val;
            updateIndex(true);
        }
        //Updates/resets index to 0
//        updateIndex(true);


        waitForStart();
        wrist.setHoverMode();
//        claw.openClaw();
//        intake.activateIntake();
        //Start Phase
        while(!isStopRequested()){

            //Drive Systems
            double forward = controller1.left_stick_y;
            double strafe = controller1.left_stick_x;
            double rotate = controller1.right_stick_x;
            robot.drive(forward,strafe,rotate);

            /*Subsystems:
                Notes:
                Use dynamic controls until constraint variables can be properly assigned/set
            * */

//            Lift Subsystem
            lift.moveLift(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());
            //Arm Subsystem
            arm.changeHeight(controller2.right_trigger.getTriggerValue() - controller2.left_trigger.getTriggerValue());
//            lift.moveLift(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());
//
//            //Claw Subsystem
//            if (controller1.a.onPress()){
//                claw.changePosition(1);
//            } else if (controller1.b.onPress()) {
//                claw.changePosition(-1);
//            }
//
            if(controller1.x.onPress() || controller2.x.onPress()){
                INCREMENT *= 10;
            }else if (controller1.y.onPress() || controller2.y.onPress()){
                INCREMENT /= 10;
            }
//
            //PID Tuning
            //Controls the value increase/decrease
            if (controller1.dpad_up.onPress() || controller2.dpad_up.onPress()){
                pidValues[index] += INCREMENT;
            }
            else if( controller1.dpad_down.onPress() || controller2.dpad_down.onPress()){
                pidValues[index] = Math.max(0, pidValues[index] - INCREMENT);
            }
//
            //Controls which value is being edited
            if (controller1.dpad_left.onPress() || controller2.dpad_left.onPress()) {
                updateIndex(false);
            }
            else if(controller1.dpad_right.onPress() || controller2.dpad_right.onPress()){
                updateIndex(true);
            }


            if(controller1.a.onPress()){
//                arm.goToBase();
                lift.goToZero();
            }
            if(controller1.b.onPress()){
//                arm.goToSpecimin();
                lift.goToTopBucket();
            }
            if(controller1.right_bumper.onPress()){
//                arm.goToDeposit();
            }



            lift.adjustPID(pidValues[0],pidValues[1],pidValues[2], pidValues[3]);
            arm.adjustPID(pidValues[4],pidValues[5],pidValues[6], pidValues[7]);
//
//
//            //Data Telemetry
            telemetry.addLine("Current Position Data");
//            telemetry.addData("Lift Motor Position: ",lift.getPosition());
            telemetry.addData("Arm Rotation: ",arm.getRotation());
//            telemetry.addData("Claw Servo Position: ",claw.getPosition());
//
            //PID Tuning Information
            telemetry.addLine("PID Tuning Information:");
            telemetry.addLine("X/Y to Increase/Decrease [INCREMENT] amount");
            telemetry.addLine("DpadUp/DpadDown to Increase/Decrease current Selected PID by said [INCREMENT] amount");
            telemetry.addLine("DpadLeft/DpadRight to switch between PID values");
            telemetry.addData("Current [INCREMENT] value: ",INCREMENT);
            telemetry.addLine(buildPIDString());

            telemetry.addLine(arm.toString());
//
            telemetry.addData("Lift Position: ",lift.getPosition());
            telemetry.addData("Power: ",arm.update());
            telemetry.addData("Target position", lift.getTargetPosition());
            telemetry.addLine(lift.getArmCurrent());
            telemetry.update();
            controller1.update();
            controller2.update();
            lift.update();
            arm.update();
        }
    }

    public void updateIndex(boolean increase){
        if (increase){
            index = (index+1) % pidValues.length;
        }else{
            index = (pidValues.length + index - 1) % pidValues.length;
        }
    }

    public String buildPIDString(){
        String result = "PID data: Line 1 = Lift PID, Line 2 = Arm PID";
        for(int i=0; i<pidValues.length; i++){
            if (i%4 == 0) {
                result += "\n";
            }
            if (i==index){
                result += "[" + pidValues[i] + "]";
            }else{
                result += " " + pidValues[i] + " ";
            }

        }
        return result;
    }
}
