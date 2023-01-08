package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagDemo;

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
        boolean bumper = false;
//        boolean leftBumper = false;
//        boolean rightBumper = false;
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
//             tevelRobot.getElevator().setElevatorMotorPower(power);
            // Show the elapsed game time and wheel power.

            telemetry.addData("position", tevelRobot.elevator.elevatorMotor.getCurrentPosition());
            telemetry.addData("power" , power);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.addData("gamepad2.rightBumper: ", gamepad2.right_bumper);
            telemetry.addData("rightBumper: ",bumper);
            if(gamepad2.b){
                tevelRobot.elevator.setElevatorPosition(tevelRobot.elevator.firstFloor);
            }
            if (gamepad2.a){
                tevelRobot.elevator.setElevatorPosition(tevelRobot.elevator.secondFloor);
            }
            if (gamepad2.x){
                tevelRobot.elevator.setElevatorPosition(tevelRobot.elevator.thirdFloor);
            }
            if (gamepad2.dpad_down){
                tevelRobot.elevator.setElevatorPosition(tevelRobot.elevator.groundFloor);
            }
          //  bumper = gamepad2.right_bumper || gamepad2.left_bumper;

            if (bumper == false && (gamepad2.right_bumper || gamepad2.left_bumper )) {
                gamepad2.rumble(1000);
                tevelRobot.getElevator().moveHands();

            }
//            if (bumper == false && gamepad2.left_bumper){
//                tevelRobot.getElevator().moveHands();
//                gamepad2.rumble(1000);}
//
//
//                {
//            rightBumper = gamepad2.right_bumper;
            bumper = gamepad2.right_bumper || gamepad2.left_bumper;
            if (yButton == false && gamepad2.y) {
                tevelRobot.elevator.moveMainServo();
            }
            yButton = gamepad2.y;


            yButton = gamepad2.y;

            tevelRobot.elevator.setElevetorDown(gamepad2.right_trigger > 0.1);

            telemetry.update();
            }
    }
}
