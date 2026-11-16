package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.NectarShooter;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.TestBench;

@TeleOp
public class Teleopmain extends OpMode {

    private Shooter pollenShooter;
    private NectarShooter nectarShooter;

    private DcMotorEx pollenmotor;

    private RobotHardwareMap robotHardwareMap;
    public boolean running;
    public boolean runPollen=false;
    public boolean runNectar=false;
    public boolean runIntake =false;
    double motorSpeed = .2;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    Intake intake;

    @Override
    public void init(){
        robotHardwareMap = new RobotHardwareMap(hardwareMap,this);
        robotHardwareMap.initialize();

        pollenShooter = new Shooter(robotHardwareMap,this);
        pollenShooter.init();

        nectarShooter = new NectarShooter(robotHardwareMap,this);
        nectarShooter.init();

        intake = new Intake(robotHardwareMap,this);
        intake.initialize();

        pollenmotor = pollenShooter.getMotor();
//        pollenmotor = hardwareMap.get(DcMotorEx.class,      "launcher_pollen");
//        nectarShooter.init(hardwareMap,"launcher_nectar");
//        intake.initialize();

    }
    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        //Control State
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            if (this.motorSpeed <= 1) {
                this.motorSpeed += .1;
            }
        }
//        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
//            if (this.motorSpeed <= 1) {
//                this.motorSpeed += .1;
//            }
//        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            if (this.motorSpeed >= .5) {
                this.motorSpeed -= .1;
            }

        }
        if (currentGamepad1.right_trigger > .5){
            if (pollenShooter.isAtSpeed()) {
                pollenShooter.shoot();
            } else {
                pollenShooter.setMotorSpeed(motorSpeed);
            }

        }
        if (gamepad1.aWasPressed()) {
            runPollen = !runPollen;
        }
        if (gamepad1.bWasPressed()) {
            runNectar = !runNectar;
        }
        if (gamepad1.yWasPressed()) {
            runIntake = !runIntake;
        }

        //Activations
        if (runPollen){
//            pollenmotor.setPower(.1);
            pollenShooter.getMotor().setPower(motorSpeed);
            pollenShooter.setMotorSpeed(0);
        } else {
           // pollenmotor.setPower(0);
            pollenShooter.getMotor().setPower(0);
        }
        if (runNectar){
            nectarShooter.setMotorSpeed(motorSpeed);
//            nectarShooter
//            pollenShooter.setMotorSpeed(0);
        } else {
            nectarShooter.setMotorSpeed(0);
        }
        if (runIntake){
             intake.on();
//            pollenShooter.setMotorSpeed(0);
        } else {
            intake.off();
        }

        //Telemtry
        buttonState.delete(0,buttonState.length());
        buttonState.append(gamepad1.a).append(":");
        buttonState.append(gamepad1.b).append(":");
        buttonState.append(gamepad1.x).append(":");
        buttonState.append(gamepad1.y).append(":");
        buttonState.append(gamepad1.dpad_up).append(":");
        buttonState.append(gamepad1.dpad_down).append(":");
        buttonState.append(gamepad1.dpad_left).append(":");
        buttonState.append(gamepad1.dpad_right).append(":");
        telemetry.addData("gamepad:a,b,x,y,up,down,left,right",buttonState.toString());
        telemetry.addData("gamepad1",gamepad1.toString());
        telemetry.addData("gamepad2",gamepad2.toString());
//        telemetry.addData("runningPollen",pollenShooter.isRunning());
        telemetry.addData("pollenPower",pollenmotor.getPower());
//        telemetry.addData("runningNectar",nectarShooter.isRunning());
        telemetry.addData("intendedPower", this.motorSpeed);
//        telemetry.addData("nectarVelocity",robotHardwareMap.launcherNectar.getVelocity());
        telemetry.addData("pollenVelocity",pollenmotor.getVelocity());


    }
    StringBuffer buttonState = new StringBuffer();

}
