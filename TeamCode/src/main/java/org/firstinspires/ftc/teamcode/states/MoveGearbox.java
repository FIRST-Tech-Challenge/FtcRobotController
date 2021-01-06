/*
Copyright 2020 FIRST Tech Challenge Team 15317
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.states;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drive;
import java.text.SimpleDateFormat;
import java.util.Date;
import org.firstinspires.ftc.teamcode.OurState;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class MoveGearbox extends OurState {
    /* Declare OpMode members. */
    //public Drive d = null;
    private Servo g = null;
    public RobotHardware robotHardware = null;
    private double lmax = 1;
    private double lmin = .5;
    //private double lmid = .5;
    private double goal = lmin;

    public MoveGearbox(String p){
        super ();
        if (p == "brake") {
            goal = lmin;
        }

        if (p == "mesh") {
            goal = lmax;
        }
    }


    public void init(RobotHardware r) {
        robotHardware = r;
        telemetry.addData("Status", "Initialized");
        //d = robotHardware.d;
        g = robotHardware.g;

        //f.setPosition(lmax);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (running) {
            double error = 0.09;
            double increment = 0.07;
            double pos1 = g.getPosition();
            if (pos1 > goal - error && pos1 < goal + error) {
                pos1 = goal;
                running = false;
            } else if (pos1 > goal) {
                pos1 -= increment;
            } else if (pos1 < goal){
                pos1 += increment;
            }
            g.setPosition(pos1);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}