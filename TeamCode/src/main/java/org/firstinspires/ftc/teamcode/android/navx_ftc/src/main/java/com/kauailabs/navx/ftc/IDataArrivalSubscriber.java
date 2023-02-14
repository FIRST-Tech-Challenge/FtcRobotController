/* ============================================
 NavX-MXP and NavX-Micro source code is placed under the MIT license
 Copyright (c) 2015 Kauai Labs

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

package org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc;

/**
 * The IDataArrivalSubscriber interface provides a method for consumers
 * of navX-Model device data to be rapidly notified whenever new data
 * has arrived.
 *
 * Two separate "callback" functions are provided:
 *
 * - timesetampedDataReceived():  reception of sensor-timestamped data
 * - untimestampedDataReceived():  reception of data without a sensor-
 *     provided timestamp.
 *
 *  Both timestamps are at millisecond resolution.  A "system timestamp"
 *  is provided in both casees, which represents as accurately as possible
 *  the time at which this process acquired the data.
 *
 *  If available, sensor timestamps are preferred, as they are generated
 *  by a real-time system and thus have a greater accuracy than the
 *  system timestamp which is vulnerable to latencies introduced by the
 *  non-realtime Android operating system.
 *
 *  To support data retrieval of various different types of data from a
 *  single data publisher, a "kind" object is also provided which can be
 *  used to indicate the type of data being referred to in the
 *  notification.
 *
 *  This callback is invoked by the AHRS class whenever new data is r
 *  eceived from the sensor.  Note that this callback is occurring
 *  within the context of the AHRS class IO thread, and it may
 *  interrupt the thread running this opMode.  Therefore, it is
 *  very important to use thread synchronization techniques when
 *  communicating between this callback and the rest of the
 *  code in this opMode.
 *
 *  The sensor_timestamp is accurate to 1 millisecond, and reflects
 *  the time that the data was actually acquired.  This callback will
 *  only be invoked when a sensor data sample newer than the last
 *  is received, so it is guaranteed that the same data sample will
 *  not be delivered to this callback more than once.
 *
 *  If the sensor is reset for some reason, the sensor timestamp
 *  will be reset to 0.  Therefore, if the sensor timestamp is ever
 *  less than the previous sample, this indicates the sensor has
 *  been reset.
 *
 *  In order to be called back, this interface must be registered
 *  via the AHRS registerCallback() method.
 */

public interface IDataArrivalSubscriber {
    public void untimestampedDataReceived( long system_timestamp, Object kind );
    public void timestampedDataReceived( long system_timestamp, long sensor_timestamp, Object kind );
    public void yawReset();
}
