package org.firstinspires.ftc.teamcode.myUtil.threads.auto;

/*
public class Arm extends Thread{


    LinearOpMode opMode;
    MecanumHardAuto r;
    boolean down = true;
    double loc = 0;

    public Arm(MecanumHardAuto r, LinearOpMode opMode){
        this.r = r;
        this.opMode = opMode;
    }
    public void move(double loc){
        this.loc = loc;
        start();
    }


/*
    @Override
    public void run(){
        r.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (loc > 0){
            while (r.arm.getCurrentPosition() < loc && !opMode.isStopRequested()){
                r.arm.setPower(0.5);
            }
        }else {
            while (r.arm.getCurrentPosition()>-loc && !opMode.isStopRequested()){
                r.arm.setPower(-0.5);
            }
        }
//        down = !down;
        r.arm.setPower(0);
    }

}

 */
