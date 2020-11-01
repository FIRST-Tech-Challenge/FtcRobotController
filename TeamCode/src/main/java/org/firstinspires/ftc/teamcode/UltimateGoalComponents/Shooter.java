package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

class Shooter extends RobotComponent {



   private ElapsedTime runtime = new ElapsedTime();
   public DcMotor ShooterWheel;

    public final double STOP = 0;

    public Shooter(robotBase base) {
        super(base);

        ShooterWheel = base.getMapper().mapMotor("sw", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void shoot(){
        getToTargetSpeed(1);
    }

    public void getToTargetSpeed(double target_speed) {
        boolean target;
        double initEncoder;
        double finEncoder;
        double difEncoder;
        double timeRecorded;

        ShooterWheel.setPower(target_speed);
        do{
            initEncoder = ShooterWheel.getCurrentPosition();
            timeRecorded = runtime.seconds();
            while(runtime.seconds()<=timeRecorded);
            finEncoder = ShooterWheel.getCurrentPosition();

            difEncoder = finEncoder - initEncoder;

           if(difEncoder>2700){
               target = true;
           } else
           {
               target = false;
           }
        }
        while(!target);
    }

    @Override
    public void stop() {
        ShooterWheel.setPower(STOP);
    }
}
