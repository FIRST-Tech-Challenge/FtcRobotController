package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@TeleOp(name = "BioBuzz Teleop", group = "Testing") // on the DS, opmodes are sorted by gorup, then name
public class BioBuzzTeleop extends OpMode {
    
    private ElapsedTime clock = new ElapsedTime();
    private State currentState;
    
    private enum State {
        COLLECT,
        DRIVETOSIDE,
        SEARCH,
        AUTOAIM,
        MANUALAIM,
        SHOOT
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize the hardware variables.
      

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Init OK");

        // Initialize the cameras and greenzone and other things
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit START
    @Override
    public void init_loop() {
        // keep reseting odometry to 0
    }

    // Code to run ONCE when the driver hits START
    @Override
    public void start() {
        currentState = State.COLLECT;
        clock.reset();
    }

    // Code to run REPEATEDLY after the driver hits START but before they hit STOP
    // This is the main program loop
    @Override
    public void loop() {
        double throttle = gamepad1.left_stick_x;

        
        if (currentState == State.COLLECT){
            /*
            1. Activate intake system and reverse feed wheel
            2. use camera 1 to search for balls
            3. if no ball move around to find one
            4. if 4 balls detected, move to search state
            */
        } else if (currentState == State.DRIVETOSIDE){
            /*
            1. keep feedwheel reverse and turn off intake
            2. if side boolean is false move to a certain side of the hive using odometry
            3. else move to the other side of the hive using odometry
            4. once you make it to that side then move to search state
            */
        } else if (currentState == State.SEARCH) {
            /*
            1. start rotating the turret
            2. if april tag is detected then stop rotation and move to auto aim state
            3. if no april tag is detected and it has rotated a full circle then:
                set side boolean as !side
                move to drive state
            */
        } else if (currentState == State.AUTOAIM) {
            /*
            1. rotate turret until the tag is centered to the camera
            2. adjust robot position for best shot while camera is adjusting to tag
            3. if green then move to shoot state
            */
        } else if (currentState == State.MANUALAIM) {
            /*
            
            */
        } else if (currentState == State.SHOOT) {
            /*
            1. if not green go back to autoaim
            2. find flywheel speed for best shot
            3. turn on feedwheel
            4. shoot
            5. move to collect state
            */
        }

        telemetry.addData("State", currentState);
    }

    // Code to run ONCE after the driver hits STOP
    // Use to set safe shutdown position if needed, but keep it quick
    @Override
    public void stop() {
    }

}
