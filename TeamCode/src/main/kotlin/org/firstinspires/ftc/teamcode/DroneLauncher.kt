package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice

class DroneLauncher(private var op: OpMode, private var motor: HardwareDevice) {
    @Suppress("MemberVisibilityCanBePrivate")
    var isArmed: Boolean = false
        private set;

    /**
     * @return "true" if it was successfully armed or "false" if it was already armed
     */
    fun arm(): Boolean {
        TODO("Not yet implemented")
//        return if (!isArmed) {
//            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
//            true;
//        } else {
//            false;
//        }
    }

//    /**
//     * @return "true" if it was successfully armed or "false" if it was already armed
//     */
//    public fun armAsync(): Boolean { }

    /**
     * @return "true" if it was successfully disarmed or "false" if it was already disarmed
     */
    fun disarm(): Boolean {
        TODO("Not yet implemented")
        return if (isArmed) {
            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
            true;
        } else {
            false;
        }
    }

//    /**
//     * @return "true" if it was successfully disarmed or "false" if it was already disarmed
//     */
//    public fun disarmAsync(): Boolean { }

    fun launch(launchWithoutArming: Boolean?): Boolean {
        TODO("Not yet implemented")
        return if (isArmed || launchWithoutArming == true) {
            if (false /** catastrophic error */) throw Exception("this will never be called, catastrophic error");
            true;
        } else {
            false;
        }
    }

//    public fun launchAsync(launchWithoutArming: Boolean?): Boolean { }
}