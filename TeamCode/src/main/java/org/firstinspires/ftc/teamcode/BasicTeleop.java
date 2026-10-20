package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BasicTeleop", group="mai")  // on the DS, opmodes are sorted by gorup, then name
public class BasicTeleop extends OpMode {
    private ElapsedTime clock = new ElapsedTime();
    private MecanumDrive drive;
    private Localizer loc;
    private State currentState;
    List<PathPoint> list = new ArrayList<PathPoint>(); // each is "x" , "y" , "h" and is where the robot should move
    List<PathPoint> startLocs = new ArrayList<PathPoint>();
    int occ; // New Variable for which occurence in the List<String> the robot should move to.
    private PathPoint goTo;
    private final int BASETIMELIM; // The base time limit in miliseconds for PathPoint until the robot moves to the next location requested to go
    private final int SIDEON; // 0 = red team 1 = blue team
    private final int STARTON;
    
    
    private enum State {
        STOP,
        FOLLOWPATH,
        OBSERVEBALL,
        MOVETOBALL,
        CAPTUREBALL,
        MOVETOBUCKET,
        SHOOTBALL
    }
    
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize the hardware variables. 
        loc = new Localizer(hardwareMap, "gbpoc");
        drive = new MecanumDrive(hardwareMap, loc);
        currentState = State.STOP;
        occ = 0;
        addPaths();
        BASETIMELIM = 2000; // 2 seconds
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Init OK");
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit START
    @Override
    public void init_loop() {
        while(SIDEON = null){
            if (gamepad1.b){
                giveStartLocs(0);
                SIDEON = 0;
            }
            if (gamepad1.x){
                giveStartLocs(1);
                SIDEON = 1;
            }
        }
                if (gamepad1.dpad_up){
            // STARTON = 0;
            // loc.startAt.startLocs.get(STARTON).giveX();
        }
        if (gamepad1.dpad_down){
            //STARTON = 1;
            //loc.startAt.startLocs.get(STARTON).giveX();
        }
    }

    //Code to run ONCE when the driver hits START
    @Override
    public void start() {
        clock.reset();
    }

    // Code to run REPEATEDLY after the driver hits START but before they hit STOP
    // This is the main program loop
    @Override
    public void loop() {
        // read sensors here
        loc.update();
        Pose2D currentLoc = loc.getPosition();
        telemetry.addData("Current: ", currentLoc);
        telemetry.addData("\nCurrent State: ", currentState);
        double x = gamepad1.left_stick_y;   // gamepad 'up' (+y) is robot +X
        double y = -gamepad1.left_stick_x;  // gamepad 'left' (-x) is robot +y
        double r = -gamepad1.right_stick_x; // gamepad 'left' (-x) is positive rotation

        // state machines here
        if (currentState == State.STOP){
            if (gamepad1.y){
                clock.reset();
                currentState = State.FOLLOWPATH;
            }
            drive.move(x, y, r);
        }
        if (currentState == State.FOLLOWPATH){
            if((drive.moveTo(list.get(occ).giveX() , list.get(occ).giveY() , list.get(occ).giveH())) && ((occ+1) < list.size()) || (clock.milliseconds() >= list.get(occ).giveTimeLim()) && ((occ+1) < list.size())){ 
               clock.reset();
               occ++;
            }else if ((clock.milliseconds() >= list.get(occ).giveTimeLim()) && (occ == list.size()-1) || (drive.moveTo(list.get(occ).giveX() , list.get(occ).giveY() , list.get(occ).giveH()) && (occ == list.size()-1))){
                clock.reset();
                occ = 0;
                currentState = State.STOP;
            }
            drive.moveTo(list.get(occ).giveX() , list.get(occ).giveY() , list.get(occ).giveH());
            if(gamepad1.x) {
                clock.reset();
                occ = 0;
                currentState = State.STOP;
            }
        }
    }

    // Code to run ONCE after the driver hits STOP
    // Use to set safe shutdown position if needed, but keep it quick
    @Override
    public void stop() {
    
    }

    // Adds different locations for robot to attempt to go to, may not be needed for competition
    public void addPaths() {
        goTo =  new PathPoint(0,100,0,BASETIMELIM);
        list.add(goTo);
        goTo = new PathPoint(100,0,0,BASETIMELIM);
        list.add(goTo);
        goTo = new PathPoint(0,-100,0,BASETIMELIM);
        list.add(goTo);
        goTo = new PathPoint(-100,0,0,BASETIMELIM);
        list.add(goTo);
    }

    public void giveStartLocs(int side){
        if (side == 0){
            goTo = new PathPoint(null,null,null,null);
            startLocs.add(goTo);
            goTo = new PathPoint(null,null,null,null);
            startLocs.add(goTo);
        }else if (side == 1){
            goTo = new PathPoint(null,null,null,null);
            startLocs.add(goTo);
            goTo = new PathPoint(null,null,null,null);
            startLocs.add(goTo);
        }
    }

}