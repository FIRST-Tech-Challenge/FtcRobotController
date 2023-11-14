package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teeloperatiuni", group="Linear Opmode")

public class TeleOp2023 extends GlobalScope2023 {
    private VoltageSensor batteryVoltageSensor;
    double drive;
    double strafe;
    double twist;
    int offset=0;
    int stateCleste=0;
    int stateServo=0;
    int testbrat=0;
    double schimbator = 0.4;
    double testsj=0.59;
    int state=0,stateBratz=0;
    double rotpos,sjpos,c1pos,c2pos;
    double[] speeds = {
            (drive + strafe + twist)/2.0,
            (drive - strafe - twist)/2.0,
            (drive - strafe + twist)/2.0,
            (drive + strafe - twist)/2.0
    };
    GamepadEx ct1;
    GamepadEx ct2;
    ButtonReader but2r,butCleste,bratsus,bratjos,skemaprindere,adjsus,adjjos,cautator_de_viteze;

    void CLAWINGTON()
    {
        ///skemaprindere.readValue();
        /**
         * A se decomenta in caz de modificari pentru calibrare
        sjsus.readValue();
        sjjos.readValue();

         * if(bratsus.wasJustPressed())
            testbrat+=5;
        if (bratjos.wasJustPressed())testbrat-=5;
        if(sjsus.wasJustPressed())
        {
            sj.setDirection(Servo.Direction.REVERSE);
            testsj-=0.01;
        }
        if(sjjos.wasJustPressed())
        {
            sj.setDirection(Servo.Direction.REVERSE);
            testsj+=0.01;
        }

        double voltage = batteryVoltageSensor.getVoltage();
        if (adjsus.wasJustPressed())
            offset-=20;
        if(adjjos.wasJustPressed())
            offset+=20;
        if(bratsus.wasJustPressed()&&stateBratz<5)
        {
            stateBratz++;
            mb1.setPower(.9);
            mb2.setPower(.9);
        }
        if(bratjos.wasJustPressed()&&stateBratz>0)
        {
            stateBratz--;
            mb1.setPower(.4);
            mb2.setPower(.4);
        }
        //if(adjsus.wasJustPressed())stateServo=1-stateServo;
         */
        butCleste.readValue();
        if(butCleste.wasJustPressed()) {
            stateCleste = 1 - stateCleste;
            rot.setPosition(rotpos + (0.12 * stateCleste));
        }
        DeschideCleste();
        /*
        if(skemaprindere.wasJustPressed())ManevraPrindere();
        */
        /**
        A se decomenta in caz de modificari la hard
        sj.setPosition(testsj);
        mb1.setTargetPosition(testbrat);
        mb2.setTargetPosition(testbrat);
        mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb1.setPower(0.2);
        mb2.setPower(0.2);*/
    /*
        if(gamepad2.a) {
            c1.setPosition(c1pos-0.1);
            c2.setPosition(c2pos+0.1);
        }
        if(gamepad2.b) {
            c1.setPosition(c1pos+0.1);
            c2.setPosition(c2pos-0.1);
        }

     */
    }

    void ManevraPrindere()///skema cu lumina si avansatu, am scris asta la ora 3 si am uitat principii de baza ale autonomiei
    {
        state=1;
        MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition()+2000);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition()+2000);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition()+2000);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition()+2000);
        MotorFS.setPower(0.2);
        MotorFD.setPower(0.2);
        MotorSS.setPower(0.2);
        MotorSD.setPower(0.2);
        while (MotorSD.isBusy());
        state=1;
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void DeschideCleste()
    {
        but2r.readValue();
        if(but2r.wasJustPressed())
        {
            state=1-state;
            //telemetry.addData("agaga ", state);
            if(state==1)
            {
                //if(state==0 && c1.getPosition()==c1pos && c2.getPosition()==c2pos) {
                c1.setDirection(Servo.Direction.REVERSE);
                c2.setDirection(Servo.Direction.REVERSE);
                c1.setPosition(c1pos + 0.65);
                c2.setPosition(c2pos - 0.65);
                //}
            }
            if(state==0)
            {
                c1.setDirection(Servo.Direction.FORWARD);
                c2.setDirection(Servo.Direction.FORWARD);
                c1.setPosition(c1pos);
                c2.setPosition(c2pos);
            }
        }
    }
    void MOVINGTON() {
        cautator_de_viteze.readValue();
        if(cautator_de_viteze.wasJustPressed())schimbator=1.10-schimbator;
        drive  = -gamepad1.left_stick_y*schimbator;//-gamepad1.left_stick_y*0.3-
        strafe = gamepad1.left_stick_x*schimbator;//gamepad1.left_stick_x*0.3b
        twist  = (gamepad1.right_trigger-gamepad1.left_trigger)/2.5 ;
        speeds[0]=(drive + strafe + twist);//FS
        speeds[1]=(drive - strafe - twist);//FD
        speeds[2]=(drive - strafe + twist);//SS
        speeds[3]=(drive + strafe - twist);//SD
        ///telemetry.addData("axa y", gamepad1.right_stick_y);
        ///telemetry.addData("axa x", gamepad1.right_stick_x);
        ///telemetry.update();
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        // apply the calculated values to the motors.
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        Initialise();
        batteryVoltageSensor=hardwareMap.voltageSensor.iterator().next();
        waitForStart();
        if(isStopRequested())
            return;
        ct1= new GamepadEx(gamepad1);
        ct2= new GamepadEx(gamepad2);
        but2r= new ButtonReader(ct2, GamepadKeys.Button.X);
        butCleste= new ButtonReader(ct2, GamepadKeys.Button.A);
        cautator_de_viteze=new ButtonReader(ct1, GamepadKeys.Button.B);


        sj.setDirection(Servo.Direction.REVERSE);
        rot.setPosition(0.47);
        sj.setPosition(0.5);
        c1.setPosition(0.5);
        c2.setPosition(0.5);
        rotpos = rot.getPosition()+0.06;
        c1pos=c1.getPosition();
        c2pos=c2.getPosition();
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rot.setDirection(Servo.Direction.REVERSE);
        while (opModeIsActive()) {
            MOVINGTON();
            CLAWINGTON();
            if(gamepad2.dpad_up)
            {
                mb1.setPower(0.50);
                mb2.setPower(0.50);
            }
            else if(gamepad2.dpad_down)
            {
                mb1.setPower(-0.25);
                mb2.setPower(-0.25);
            }
            else
            {
                mb1.setPower(0);
                mb2.setPower(0);
            }
            telemetry.addData("pos mb2", mb2.getCurrentPosition());
            telemetry.addData("pos mb1", mb1.getCurrentPosition());
            telemetry.addData("offset", MotorFS.getCurrentPosition());
            telemetry.update();
        }
    }
}