/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.openftc.easyopencv;

import android.hardware.camera2.CameraCharacteristics;

public interface OpenCvInternalCamera2 extends OpenCvCamera
{
    enum CameraDirection
    {
        FRONT(CameraCharacteristics.LENS_FACING_FRONT),
        BACK(CameraCharacteristics.LENS_FACING_BACK);

        public int id;

        CameraDirection(int id)
        {
            this.id = id;
        }
    }

    //-----------------------------------------------------------------------
    // ISO
    //-----------------------------------------------------------------------

    /***
     * Get the minimum gain (ISO) supported by the image sensor
     *
     * @return see above
     */
    int getMinSensorGain();

    /***
     * Get the maximum gain (ISO) supported by the image sensor
     *
     * @return see above
     */
    int getMaxSensorGain();

    /***
     * Set the gain (ISO) of the image sensor
     * Must be between {@link #getMinSensorGain()}
     * and {@link #getMaxSensorGain()}
     *
     * @param iso the gain (ISO) the image sensor should use
     */
    void setSensorGain(int iso);

    //-----------------------------------------------------------------------
    // White Balance
    //-----------------------------------------------------------------------

    enum WhiteBalanceMode
    {
        MANUAL,
        AUTO,
        CLOUDY_DAYLIGHT,
        DAYLIGHT,
        FLUORESCENT,
        INCANDESCENT,
        SHADE,
        TWILIGHT,
        WARM_FLUORESCENT
    }

    /***
     * Gives the camera's auto white balance routine a hint about
     * what type of scene is being worked with
     *
     * @param whiteBalanceMode the type of scene being worked with
     */
    void setWhiteBalanceMode(WhiteBalanceMode whiteBalanceMode);

    /***
     * Locks the camera's auto white balance routine right where it is
     *
     * @param locked whether or not the auto white balance routine should be locked
     */
    void setAutoWhiteBalanceLocked(boolean locked);

    //-----------------------------------------------------------------------
    // Exposure
    //-----------------------------------------------------------------------

    enum ExposureMode
    {
        AUTO,
        MANUAL
    }

    /***
     * Sets the exposure mode the image sensor should operate in
     *
     * @param exposureMode the exposure mode the image sensor should operate in
     */
    void setExposureMode(ExposureMode exposureMode);

    /***
     * Locks / unlocks the auto exposure routine. Changes
     * to exposure compensation will still take effect if
     * the exposure has been locked.
     *
     * Note: this setting only applies in {@link ExposureMode#AUTO}
     *
     * @param locked whether the auto exposure routine should be locked
     */
    void setAutoExposureLocked(boolean locked);

    /***
     * Get the maximum exposure compensation value
     * supported by the auto exposure routine.
     *
     * @return see above
     */
    int getMinAutoExposureCompensation();

    /***
     * Get the maximum exposure compensation value
     * supported by the auto exposure routine.
     *
     * @return see above
     */
    int getMaxAutoExposureCompensation();

    /***
     * Set the exposure compensation value of the auto exposure
     * routine. This can allow for (limited) relative exposure
     * adjustment from the automatically determined value.
     *
     * NOTE: this setting only applies in {@link ExposureMode#AUTO}
     *
     * @param aeCompensation see above
     */
    void setAutoExposureCompensation(int aeCompensation);

    /***
     * Set the exposure time the image sensor should use when capturing images.
     *
     * NOTE: this setting only applies in {@link ExposureMode#MANUAL}
     *
     * @param denominator the denominator of a traditional fractional exposure time notation
     *                    e.g. if you want to set exposure to 1/250s, this param should be 250
     */
    void setExposureFractional(int denominator);

    /***
     * Set the exposure time the image sensor should use when capturing images.
     *
     * NOTE: this setting only applies in {@link ExposureMode#MANUAL}
     *
     * @param nanos the amount of time the sensor should expose for, in nanoseconds
     */
    void setExposureNanos(long nanos);

    //-----------------------------------------------------------------------
    // Focus
    //-----------------------------------------------------------------------

    enum FocusMode
    {
        CONTINUOUS_AUTO_PICTURE,
        CONTINUOUS_AUTO_VIDEO,
        MANUAL
    }

    /***
     * Sets the focus mode the image sensor should operate in
     *
     * @param focusMode the focus mode the image sensor should operate in
     */
    void setFocusMode(FocusMode focusMode);

    /***
     * The minimum distance the camera can focus at, in diopters.
     * See {@link #setFocusDistance(float)}
     *
     * @return The minimum distance the camera can focus at, in diopters
     */
    float getMinFocusDistance();

    /***
     * Set the distance the camera should focus at in diopters. A diopter is 1/meters.
     * For instance to focus at 13cm, you want to focus at 1/0.13 diopters.
     *
     * The reason for this is that it makes representing focusing at infinity very easy
     * (to focus at infinity just set 0 diopters)
     *
     * NOTE: this setting only applies in {@link FocusMode#MANUAL}
     *
     * @param diopters See above. Must be between 0 and {@link #getMinFocusDistance()}
     */
    void setFocusDistance(float diopters);

    //-----------------------------------------------------------------------
    // Misc.
    //-----------------------------------------------------------------------

    /***
     * Set a specific number of frames per second the image sensor should capture.
     *
     * Note that your pipeline may not be fast enough to keep up with what you ask the sensor
     * to send, which will result in your pipeline FPS not matching what you set here.
     *
     * For instance, suppose you request 24FPS, but your pipeline requires an amount of
     * compute time that results in it only being able to run at 15FPS. In that case, your
     * pipeline will still run at 15FPS.
     *
     * Conversely, if your pipeline is capable of processing 100 frames per second, but
     * you request only 15FPS from the sensor, your pipeline will run at 15FPS.
     *
     * NOTE: this setting only applies in {@link ExposureMode#MANUAL}
     *
     * @param sensorFps see above
     */
    void setSensorFps(int sensorFps);

    /***
     * Set whether or not the camera's flash should be
     * put into flashlight ("torch") mode, i.e. where
     * it is always on.
     *
     * @param enabled see above
     */
    void setFlashlightEnabled(boolean enabled);
}
