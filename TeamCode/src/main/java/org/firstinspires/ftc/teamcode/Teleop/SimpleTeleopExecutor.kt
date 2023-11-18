package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import java.io.BufferedReader
import java.io.FileReader


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive TeleOp for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
// Autonomous
@TeleOp(name = "SimpleTeleopExecutor", group = "Linear Opmode")
class SimpleTeleopExecutor : DriveMethods() {
    override fun runOpMode() {
        // Setup Odometry :)
        // Wait for the game to start (driver presses PLAY)
        initMotorsSecondSimple()

        waitForStart()
        BufferedReader(FileReader("/sdcard/FIRST/training/SimpleRecordTest.txt")).use { br ->
            var line: String?
            while (br.readLine().also { line = it } != null) {
                var splitLine = line!!.split(":");
                if(splitLine.size>1){
                    var command = splitLine[0];
                    var parameter = splitLine[1];
                    when(command){
                        "motorFL.power"->motorFL!!.power=parameter.toDouble();
                        "motorBL.power"->motorBL!!.power=parameter.toDouble();
                        "motorFR.power"->motorFR!!.power=parameter.toDouble();
                        "motorBR.power"->motorBR!!.power=parameter.toDouble();
                        "sleep"->sleep(parameter.toLong());
                    }
                }
            }
        }


    }
}
