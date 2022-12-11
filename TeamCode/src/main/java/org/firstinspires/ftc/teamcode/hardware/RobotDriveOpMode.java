package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RobotDriveOpMode", group = "Linear Opmode")
public class RobotDriveOpMode extends LinearOpMode {
    double xAxis;
    double yAxis;
    double rot;

    private ElapsedTime runtime = new ElapsedTime();
  //  private Elevator elevator = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        elevator = new Elevator(hardwareMap, telemetry);
        TevelRobot tevelRobot = new TevelRobot(this);
        double power;
        boolean bButton = false;
        boolean yButton = false;
       tevelRobot.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        Wheels w = new Wheels(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            xAxis = gamepad1.left_stick_x;
            yAxis = -gamepad1.left_stick_y;
            rot = gamepad1.right_stick_x;

            tevelRobot.getWheels().driveByJoystick(xAxis, yAxis, rot);

            telemetry.addData("Left JoyStick", "X: %f, Y: %f", xAxis, yAxis);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.update();



            power = -gamepad2.left_stick_y;
             tevelRobot.getElevator().setElevatorMotorPower(power);
            // Show the elapsed game time and wheel power.
            telemetry.addData("power" , power);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("gamepad2.b: ", gamepad2.b);
            telemetry.addData("bButton: ", bButton);
            if (bButton == false && gamepad2.b) {
                tevelRobot.getElevator().moveHands();

            }
            telemetry.addData("clawstate ", tevelRobot.getElevator().clawState);


            bButton = gamepad2.b;

            if (yButton == false && gamepad2.y) {
                tevelRobot.getElevator().moveMainServo();
            }
            yButton = gamepad2.y;

            telemetry.update();
        }
    }
}
