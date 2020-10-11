/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates;

/**
 * This interface is the lowest level asynchronous Java Interface that we are going to use in Darbots-FTC-Lib.
 * Since Darbots-FTC-Lib is a library designed with the idea of asynchronous operations, in the main program loop, every sensor's updateStatus() method is called.
 * @author David Cao
 */
public interface RobotNonBlockingDevice {

    /**
     * This method checks if a sensor is at work.
     * @return If the sensor has a job at hand, returns true / otherwise returns false
     */
    boolean isBusy();

    /**
     * This method lets the sensor update its work / job status.
     * If the sensor is data-oriented (means it obtains data from some means), it also tries to obtain the new data and put that into its local storage.
     */
    void updateStatus();

    /**
     * This method blocks the main program loop.
     * This method lets the sensor keep updating its status until it is no longer busy (all assigned jobs are finished)
     */
    void waitUntilFinish();
}
