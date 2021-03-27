package com.technototes.library.hardware.sensor.encoder;


import com.technototes.library.hardware.Sensored;

/** Interfaces for encoders to use
 * @author Alex Stedman
 */
public interface Encoder extends Sensored {
    /** zero the encoder
     *
     */
    void zeroEncoder();

}
