package com.technototes.subsystem;

/** Interface for slide subsystems
 * @author Alex Stedman
 */
public interface SlideSubsystem extends PositionalSubsystem{
    /** Extend the slide
     *
     */
    void extend();

    /** Retract the slide
     *
     */
    void retract();
}
