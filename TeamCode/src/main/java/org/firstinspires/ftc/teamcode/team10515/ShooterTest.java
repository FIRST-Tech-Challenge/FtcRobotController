package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.team10515.control.ShooterPhysics;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;


@TeleOp(name = "Shooter Test", group = "Test")

public class ShooterTest extends OpMode{
    public double shooterSpeed = 0;
    public boolean moveShooter = false;

//    private static EnhancedGamepad enhancedGamepad1;

    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;

    public ShooterPhysics shooterPhysics;

    UGMapTest robot = new UGMapTest();
    @Override
    public void start(){
        telemetry.addData("Started", "Ready for Command");
        telemetry.update();
    }

    @Override
    public void init(){
        /* Initialize the hardware map*/
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Storm Trooper");    //
        updateTelemetry(telemetry);

        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        shooterPhysics = new ShooterPhysics();
    }

    @Override
    public void loop() {
//        super.loop();

//        if(getEnhancedGamepad1().isA()){
//            shooterSpeed = ShooterStateMachine.State.IDLE;
//        }
//        else if(getEnhancedGamepad1().isY()){
//            shooterSpeed = ShooterStateMachine.State.SPEED1;
//        }

//        getShooter().getStateMachine().updateState(shooterSpeed);

        // Trial 1

//        if(gamepad1.a){
//            shooterSpeed = 0.25;
//        }
//        else if(gamepad1.b){
//            shooterSpeed = 0.5;
//        }
//        else if(gamepad1.x){
//            shooterSpeed = 0.75;
//        }
//        else if(gamepad1.y){
//            shooterSpeed = 1;
//        }
//        else if(gamepad1.right_bumper){
//            shooterSpeed = 0;
//        }

        /*
        // Trial 2

        if (getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
                telemetry.addLine("a pressed");
                shooterSpeed += 0.1;
                btnPressedA.reset();
        } else if (getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250) {
//            if(currentTime-lastTimeY >= 250) {
                telemetry.addLine("y pressed");
                shooterSpeed -= 0.1;
                btnPressedY.reset();
//                lastTimeY = currentTimeMillis();
//            }
        } else if (getEnhancedGamepad1().isB()) {
            shooterSpeed = 0;
        } else if (getEnhancedGamepad1().isX()) {
            shooterSpeed = 0.5;
        }
        */

        // Trial 3

//        if(moveShooter) {
//            shooterSpeed = gamepad1.right_trigger;
//        }
//        else{
//            shooterSpeed = 0;
//        }
//
//        if(gamepad1.a){
//            moveShooter = !moveShooter;
//        }

        // Trial 4
        shooterSpeed = shooterPhysics.getShooterSpeed(1.444625, 0.9, 1);

        robot.Shooter1.setPower(shooterSpeed);
        robot.Shooter2.setPower(shooterSpeed);

        telemetry.addLine("Shooter speed: " + shooterSpeed);
        telemetry.addLine("Shooter angle: " + shooterPhysics.getShooterAngle(1.444625, 0.9));

        currentTime = currentTimeMillis();
    }

}
