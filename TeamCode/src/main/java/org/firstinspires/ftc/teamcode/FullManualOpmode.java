package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@SuppressWarnings("unused")
@TeleOp(name = "Full Manual", group = "Testing") // on the DS, opmodes are sorted by gorup, then name
public class FullManualOpmode extends OpMode {

    private ElapsedTime clock = new ElapsedTime();
    private MecanumDrive drive;
    private Localizer loc;
    private IntakeDriver intake;
    private TransferDriver transfer;
    // private Turretdriver turret;
    //private ServoMotorController turret;
    private MotorPowerController flywheel;
    private State currentState;
    private enum State {
        GO
    }
    
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize the hardware variables. 
        loc = new Localizer(hardwareMap, "gbpoc");
        drive = new MecanumDrive(hardwareMap, loc);
        intake = new IntakeDriver(hardwareMap, "intake"); 
        transfer = new TransferDriver(hardwareMap, "transfer", "feed"); 
        flywheel = new MotorPowerController(hardwareMap, "flywheel");
        // turret = new TurretDriver(hardwareMap, "turretServo");    

        currentState = State.GO;
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Init OK");
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit START
    @Override
    public void init_loop() {
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
        telemetry.addData("State",currentState);
        int currX = (int)currentLoc.getX(DistanceUnit.MM);
        int currY = (int)currentLoc.getY(DistanceUnit.MM);
        int theta = (int)currentLoc.getHeading(AngleUnit.DEGREES);
        String position = String.format("x=%6d  y=%6d  r=%6d", currX, currY, theta);
        telemetry.addData("Position",position);
        double x = -gamepad1.left_stick_y;   // gamepad 'up' (-y) is robot +X
        double y = -gamepad1.left_stick_x;  // gamepad 'left' (-x) is robot +y
        double r = -gamepad1.right_stick_x; // gamepad 'left' (-x) is positive rotation

        // STATE MACHINE START
       if (currentState == State.GO){
            drive.move(x, y, r);
            //intake.run();
            transfer.collect();
            flywheel.setPower(0.75);
            if(gamepad1.a){
                transfer.loadTurret();
            }
            if (gamepad1.b){
                drive.moveTo(0,0,0);
            }
        }
        // STATE MACHINE END
    }

    // Code to run ONCE after the driver hits STOP
    // Use to set safe shutdown position if needed, but keep it quick
    @Override
    public void stop() {
        drive.stop();
    }
}
