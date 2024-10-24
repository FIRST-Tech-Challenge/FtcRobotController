package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="Robot: Teleop", group="Robot")
public class RobotTeleopMecanumDrive extends OpMode{

    /* Declare OpMode members. */
    public Mecanum drive;
    public Drawer drawer;
    public Elevator elevator;
    public SpecimanArm specimanArm;
    static final double DRIVE_SPEED = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        drive = new Mecanum(hardwareMap);
        drive.setTelemetry(telemetry);
        drive.setDriveSpeed(DRIVE_SPEED);
        drive.setDebug(true);

        drawer = new Drawer(hardwareMap);
        //elevator = new Elevator(hardwareMap);
        specimanArm = new SpecimanArm(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        telemetry.update();
    }
    */

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.update(gamepad1);
        drawer.update(gamepad2);
        specimanArm.update(gamepad2);
        //elevator.update(gamepad2);
        testingValues(gamepad2);
    }

    public void testingValues(Gamepad gamepad) {
        if(gamepad.a)
            drawer.setJointPosition(gamepad.left_stick_y * 0.5 + 0.5); // in 0.25, out 1
        if(gamepad.b)
            drawer.setClawPosition(gamepad.left_stick_y * 0.5 + 0.5); // open 0, close 0.75
        if(gamepad.y)
            specimanArm.setPosition(gamepad.left_stick_y * 0.5 + 0.5);
        telemetry.addData("Servo Position:", gamepad.left_stick_y * 0.5 + 0.5);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}