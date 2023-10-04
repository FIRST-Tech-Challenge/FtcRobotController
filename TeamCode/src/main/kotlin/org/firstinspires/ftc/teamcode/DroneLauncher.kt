package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice

class DroneLauncher(private var op: OpMode, private var motor: HardwareDevice) {
    var isArmed: Boolean = false
        private set;

    /**
     * @return "true" if it was successfully armed or "false" if it was already armed
     */
    public fun arm(): Boolean {
        TODO("Not yet implemented")
        if (!isArmed) {
            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
            return true;
        } else {
            return false;
        }
    }

//    /**
//     * @return "true" if it was successfully armed or "false" if it was already armed
//     */
//    public fun armAsync(): Boolean { }

    /**
     * @return "true" if it was successfully disarmed or "false" if it was already disarmed
     */
    public fun disarm(): Boolean {
        TODO("Not yet implemented")
        if (isArmed) {
            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
            return true;
        } else {
            return false;
        }
    }

//    /**
//     * @return "true" if it was successfully disarmed or "false" if it was already disarmed
//     */
//    public fun disarmAsync(): Boolean { }

    public fun launch(launchWithoutArming: Boolean?): Boolean {
        TODO("Not yet implemented")
        if (isArmed || launchWithoutArming == true) {
            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
            return true;
        } else {
            return false;
        }
    }

//    public fun launchAsync(launchWithoutArming: Boolean?): Boolean { }
}