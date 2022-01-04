package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Disabled
@Autonomous(name="Move: Left w/ Timer", group="Move")
public class MoveLeftTime extends OpMode{

    private int stateMachineFlow;
    MecanumDrive robot = new MecanumDrive();
    Intake intake      = new Intake();
    Lift lift          = new Lift();
    Placing placing    = new Placing();
    SkystoneCam cam    = new SkystoneCam();

    double time;
    private boolean wasTimeIncreased;
    private boolean wasTimeDecreased;
    // controls how long the robot waits before moving
    private double waitTime;
    private ElapsedTime     runtime = new ElapsedTime();
    /***********************************
     *
     * This program starts with the robot's intake facing the center
     *
     ***********************************/
    /*@Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 25000;

        waitTime = 0;

        robot.init(hardwareMap);
        //intake.init(hardwareMap);
        //lift.init(hardwareMap);
        //placing.init(hardwareMap);
        //cam.init(hardwareMap);

        stateMachineFlow = 0;
    }
    public void init_loop(){
        if (gamepad2.dpad_up){
            wasTimeIncreased = true;
        }else if (!gamepad2.dpad_up && wasTimeIncreased){
            waitTime = waitTime + 5;
            if (waitTime == 25){
                waitTime = 0;
            }
            wasTimeIncreased = false;
        }
        if (gamepad2.dpad_down){
            wasTimeDecreased = true;
        }else if (!gamepad2.dpad_down && wasTimeDecreased){
            waitTime = waitTime - 5;
            if (waitTime == - 5){
                waitTime = 20;
            }
            wasTimeDecreased = false;
        }
        telemetry.addData("Wait Time",waitTime);
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (stateMachineFlow) {
            case 0:
                runtime.reset();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 1:
                //move under the bridge
                while (waitTime > runtime.time() - time){
                    telemetry.addData("wait time", waitTime);
                    telemetry.addData("time elapsed", runtime.time() - time);
                    telemetry.update();
                }
                robot.sideDrive(.65,-25);
                stateMachineFlow++;
                break;
        }
    }
}
*/