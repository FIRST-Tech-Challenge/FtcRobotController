package org.firstinspires.ftc.teamcode;
//import ftc.electronvolts.statemachine.StateMachine;
import com.github.pmtischler.base.StateMachine;
import com.github.pmtischler.base.StateMachine.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "rover_autonomous", group = "LAIMO")
public class DemoAuto extends OpMode {
    double tStart;
    private DcMotor FR, FL, BL, BR;
    // private Servo Doink, Doink2, X_swing, Y_swing;
    private State1 state1;
    private State2 state2;
    private State3 state3;
    private State4 state4;
    public StateMachine machine;
    private Range rangeSensor1;
    boolean state_2 = false;
    boolean state_3 = false;

    //private State machine;
    @Override
    public void init() {

        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
// BR = hardwareMap.dcMotor.get("BR");
// BL = hardwareMap.dcMotor.get("BL");
// Doink = hardwareMap.servo.get("Doink");
// Doink2 = hardwareMap.servo.get("Doink2");
// X_swing = hardwareMap.servo.get("X_swing");
// Y_swing = hardwareMap.servo.get("Y_swing");
// Lift = hardwareMap.dcMotor.get("Lift");
// while (opModeIsActive()) {
        state1 = new DemoAuto.State1();
        state2 = new DemoAuto.State2();
        state3 = new DemoAuto.State3();
        machine = new StateMachine(state1);

        FR.setDirection(DcMotor.Direction.REVERSE);
//.setDirection(DcMotor.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// set left motor to run to target encoder position and stop with brakes on.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    //run vuforia and find column
    public class State1 implements StateMachine.State {

        @Override
        public void start() {

            tStart = time;
            update();
        }

        @Override
        public StateMachine.State update() {
/* if ((time - tStart) < 10) {
Doink.setPosition(0);
Doink2.setPosition(1);
X_swing.setPosition(.8);
}*/
            FL.getCurrentPosition();
            FR.getCurrentPosition();

            FL.setTargetPosition(1000);
            FR.setTargetPosition(1000);
            FR.setPower(.25);
            FL.setPower(.25);

            while (FL.isBusy() ) {
                telemetry.addData("encodeValueFL", FL.getCurrentPosition()); /*+ "busy" + FL.isBusy()*/;
                telemetry.addData("encodeValueFR", FR.getCurrentPosition());

                telemetry.update();
            }

            return state1;

        }

    }

    public class State2 implements StateMachine.State {

        @Override
        public void start() {

            state_2 =true;

            if(state_2 = true) {
                telemetry.addData("state_2", state_2);
                telemetry.update();
            }
            update();
// tStart = time;
        }

        @Override
        public StateMachine.State update() {
/* if ((time - tStart) < 20) {
//color sensor
if(ballIsBlue() == true)
{
Y_swing.setPosition(0);
}
if(ballIsBlue() == false)
{
Y_swing.setPosition(1);
}
}*/
            FL.getCurrentPosition();
            FR.getCurrentPosition();

            FL.setTargetPosition(-1000);
            FR.setTargetPosition(-1000);
            FR.setPower(.25);
            FL.setPower(0.80);

            return state3;
        }
    }

    public class State3 implements StateMachine.State {

        @Override
        public void start() {

// state_3 =true;
            if(state_3 = true) {
                telemetry.addData("state_3", state_3);
                telemetry.update();
            }
            update();
        }

        @Override
        public StateMachine.State update() {
/* if ((time - tStart) < 30) {
FL.setPower(1);
FR.setPower(1);
BL.setPower(-1);
BR.setPower(1);
}*/
            FL.getCurrentPosition();
            FR.getCurrentPosition();

            FL.setTargetPosition(1000);
            FR.setTargetPosition(1000);
            FR.setPower(0);
            FL.setPower(.25);

            return state4;
        }
    }

    public class State4 implements StateMachine.State {

        @Override
        public void start() {
            tStart = time;
// update();
        }

        @Override
        public StateMachine.State update() {
/* if ((time - tStart) < 40) {
FL.setPower(1);
BL.setPower(1);
}*/
            FL.getCurrentPosition();
            FR.getCurrentPosition();

            FL.setTargetPosition(-1000);
            FR.setTargetPosition(-1000);
            FR.setPower(-.25);
            FL.setPower(-.25);

            return null;
        }
    }

    public boolean ballIsBlue() {
        return true;
    }

    public void loop() {
        machine.update();
    }

/*public StateMachine machine;
public State1 state1;
public State2 state2;
public State3 state3;
public State4 state4;*/
}