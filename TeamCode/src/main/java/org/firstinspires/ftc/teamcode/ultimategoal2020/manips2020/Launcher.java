package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher implements EbotsManip2020 {

    private DcMotorEx launcherMotor;
    final int  HIGH_GOAL = 1347;
    final int  LOW_GOAL = 1250;
    final int  POWER_SHOTS = 1303;

    public Launcher(HardwareMap hardwareMap){
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getPower(){
        return this.launcherMotor.getPower();
    }

    public double getVelocity(){
        return this.launcherMotor.getVelocity();
    }

    @Deprecated
    public void setPower(double targetPower){
        this.launcherMotor.setPower(targetPower);
    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        // ************     LAUNCHER   **********************
        // Set the speed for the shooter
        //  Y - HIGH GOAL
        //  B - POWER SHOTS
        //  A - LOW GOAL
        //  X - STOP
        if(gamepad.y){
            launcherMotor.setVelocity(HIGH_GOAL);
//            launcher.setPower(HIGH_GOAL);
        }else if(gamepad.b){
            launcherMotor.setVelocity(POWER_SHOTS);
//            launcher.setPower(POWER_SHOTS);
        }else if(gamepad.a){
            launcherMotor.setVelocity(LOW_GOAL);
//            launcher.setPower(LOW_GOAL);
        }else if(gamepad.x){
            launcherMotor.setPower(0);
        }
    }

    @Override
    public void stop() {
        launcherMotor.setPower(0);
    }

    public void startLauncher(){
        launcherMotor.setVelocity(HIGH_GOAL);
    }

    @Deprecated
    public void setVelocity(double targetVelocity){
        launcherMotor.setVelocity(targetVelocity);
    }

}
