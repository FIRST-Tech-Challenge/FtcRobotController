/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.external;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Locale;

/**
 * Instances of {@link Telemetry} provide a means by which data can be transmitted from the
 * robot controller to the driver station and displayed on the driver station screen.
 *
 * <p>Simple use of {@link Telemetry} consists of a series of {@link #addData(String, String, Object...)
 * addData()} calls, followed by a call to {@link #update()}. For example:</p>
 *
 * <pre>
 *     // LinearOpMode
 *     telemetry.addData("count", currentCount);
 *     telemetry.addData("elapsedTime", "%.3f", elapsedSeconds);
 *     telemetry.update();
 * </pre>
 *
 * <p>In the 2015/16 season, the call to {@link #update()} was not required; now, however,
 * in a {@link LinearOpMode}, unless {@link #update()} is called, nothing will appear on the
 * driver station screen. In other, loop-based OpModes, {@link #update()} continues to be called
 * automatically at the end of {link OpMode#loop()} and {link OpMode#init_loop()}; no call to
 * {@link #update()} is required in loop-based OpModes.</p>
 *
 * <pre>
 *     // loop-based OpMode
 *     telemetry.addData("count", currentCount);
 *     telemetry.addData("elapsedTime", "%.3f", elapsedSeconds);
 * </pre>

 * <p>By default (but see {@link #setAutoClear(boolean) setAutoClear()}), data is cleared from the
 * telemetry after each call to {@link #update()}; thus, you need to issue {@link #addData(String,
 * Object) addData()} for the entire contents of the telemetry screen on each update cycle.
 * This behavior is just as it was in the 2015/16 season.</p>
 *
 * <p>A more complicated use of {@link Telemetry} might have different parts of the program update
 * different items on the display in a decoupled, decentralized manner. Such situations might
 * usefully avail themselves of turning off the auto-clear setting. For example:</p>
 *
 * <pre>
 *     telemetry.setAutoClear(false);
 *     Telemetry.Item countItem = telemetry.addData("count", 0);
 *     Telemetry.Item elapsedItem = telemetry.addData("elapsedTime", 0);
 *
 *     void onePartOfYourCode() {
 *         ...
 *         countItem.setValue(0);
 *         telemetry.update();
 *         ...
 *     }

 *     void anotherPartOfYourCode() {
 *         ...
 *         elapsedItem.setValue("%.3f", elapsedSeconds);
 *         telemetry.update();
 *         ...
 *     }
 * </pre>
 *
 * <p>In this way different parts of the code can update only a portion of the telemetry screen
 * without disturbing other parts.</p>
 *
 * <p>Below the list of caption, value, pairs, on the screen a {@link Telemetry} also displays
 * a short, unstructured log of messages. Use {@link Log#add(String, Object...) log().add()}
 * to add to the log. See also {@link #log()}.</p>
 *
 * <p>Actual transmission to the driver station is throttled to avoid use of excessive bandwidth.
 * By default, transmission will occur at most every 250ms. This interval can be controlled with
 * {@link #setMsTransmissionInterval(int) setMsTransmissionInterval()}. Any {@link #update()}s which
 * occur more frequently will not be transmitted if superseded by a subsequent {@link #update()}
 * before the transmission interval expires.</p>
 *
 * <p>As mentioned above, by default, after each call to {@link #update()}, the method {@link #clear()}
 * is automatically called. Thus, in simple usage, after each {@link #update()}, you'll want to issue
 * {@link #addData(String, Object) addData()} calls to rebuild the entire driver station telemetry
 * screen that you wish to observe. This simple usage can be modified in two ways.</p>
 *
 * <p>First, the automatic issuance of calls to {@link #clear()} can be altered by means of
 * {@link #setAutoClear(boolean)}. If auto clearing is set to false, previously added telemetry
 * items remain present from {@link #update()} to {@link #update()}, but their value can still be altered
 * using {@link Item#setValue(Object) Item.setValue()}, items can be removed using {@link #removeItem(Item)},
 * and new items can be added using {@link #addData(String, Object) addData()} and {@link Item#addData(String,
 * Object) Item.addData()}.</p>
 *
 * <p>Second, telemetry items created in a functional form using {@link #addData(String, Func)} or
 * {@link Item#addData(String, Func)} are <em>not</em> automatically removed when {@link #clear()}
 * is called, either implicitly or explicitly (though they are removed when {@link #clearAll()} is
 * called). The intent of such items is allow a one-time specification of telemetry items by
 * providing a function that can <em>produce</em> a value to be displayed rather than providing the
 * actual value itself. Such functions are evaluated only when it is known that telemetry is going to
 * be transmitted to the driver station (and so a value is required). This approach can be particularly
 * useful if the acquisition of the data to be displayed is relatively expensive or time consuming, as
 * that cost is only expended when actually useful.
 *
 * In addition to one-item-per-line display on the driver station, multiple items per line can
 * be displayed by starting with a call to {@link #addLine()} and then following up with one or more
 * {@link #addData(String, Object)} calls. For example:
 *
 * <pre>
 *     telemetry.addLine()
 *          .addData("count", currentCount)
 *          .addData("elapsedTime", "%.3f", seconds);
 *     telemetry.addLine()
 *          .addData("voltage", "%.1f", getCurrentVoltage())
 *          .addData("orientation", "%s", getOrientation());
 *     telemetry.update();
 * </pre>
 *
 * <p>Items on the same line are separated by the {@link #getItemSeparator() item separator}. Caption
 * and value within an item are separated by the {@link #getCaptionValueSeparator() caption value separator}.</p>
 *
 * <p>Note: in the 2015/16 season, it was possible for {@link Telemetry} instances to be automatically
 * sorted in alphabetical order by caption. This functionality is no longer supported.</p>
 */
@SuppressWarnings("javadoc")
public interface Telemetry
{
    //----------------------------------------------------------------------------------------------
    // Core usage
    //----------------------------------------------------------------------------------------------

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link String#format(Locale, String, Object...) String.format()}
     * with the indicated format and arguments. The caption and value are shown on the driver station
     * separated by the {@link #getCaptionValueSeparator() caption value separator}. The item
     * is removed if {@link #clear()} or {@link #clearAll()} is called.
     *
     * @param caption   the caption to use
     * @param format    the string by which the arguments are to be formatted
     * @param args      the arguments to format
     * @return          an {@link Item} that can be used to update the value or append further {@link Item}s
     *
     * @see #addData(String, Object)
     * @see #addData(String, Func)
     */
    Item addData(String caption, String format, Object... args);

    /**
     * Adds an item to the end if the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link Object#toString() toString()} on the provided value
     * object. The caption and value are shown on the driver station separated by the {@link
     * #getCaptionValueSeparator() caption value separator}. The item is removed if {@link #clear()}
     * or {@link #clearAll()} is called.
     *
     * @param caption   the caption to use
     * @param value     the value to display
     * @return          an {@link Item} that can be used to update the value or append further {@link Item}s
     *
     * @see #addData(String, String, Object...)
     * @see #addData(String, Func)
     */
    Item addData(String caption, Object value);

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link Object#toString() toString()} on the object which is
     * returned from invoking valueProducer.{@link Func#value()} value()}. The caption and value are
     * shown on the driver station separated by the {@link #getCaptionValueSeparator() caption value
     * separator}. The item is removed if {@link #clearAll()} is called, but <em>not</em> if
     * {@link #clear()} is called.
     *
     * <p>The valueProducer is evaluated only if actual transmission to the driver station
     * is to occur. This is important, as it provides a means of displaying telemetry which
     * is relatively expensive to evaluate while avoiding computation or delay on evaluations
     * which won't be transmitted due to transmission interval throttling.</p>
     *
     * @param caption           the caption to use
     * @param valueProducer     the object which will provide the value to display
     * @return                  an {@link Item} that can be used to update the value or append further {@link Item}s
     *
     * @see #addData(String, String, Object...)
     * @see #addData(String, Object)
     * @see #addData(String, String, Func)
     * @see #getMsTransmissionInterval()
     */
    <T> Item addData(String caption, Func<T> valueProducer);

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link String#format} on the object which is returned from invoking
     * valueProducer.{@link Func#value()} value()}. The caption and value are shown on the driver station
     * separated by the {@link #getCaptionValueSeparator() caption value separator}. The item is removed
     * if {@link #clearAll()} is called, but <em>not</em> if {@link #clear()} is called.
     *
     * <p>The valueProducer is evaluated only if actual transmission to the driver station
     * is to occur. This is important, as it provides a means of displaying telemetry which
     * is relatively expensive to evaluate while avoiding computation or delay on evaluations
     * which won't be transmitted due to transmission interval throttling.</p>
     *
     * @param caption           the caption to use
     * @param valueProducer     the object which will provide the value to display
     * @return                  an {@link Item} that can be used to update the value or append further {@link Item}s
     *
     * @see #addData(String, String, Object...)
     * @see #addData(String, Object)
     * @see #addData(String, Func)
     * @see #getMsTransmissionInterval()
     */
    <T> Item addData(String caption, String format, Func<T> valueProducer);

    /**
     * Removes an item from the receiver telemetry, if present.
     * @param item  the item to remove
     * @return      true if any change was made to the receive (ie: the item was present); false otherwise
     */
    boolean removeItem(Item item);

    /**
     * Removes all items from the receiver whose value is not to be retained.
     * @see Item#setRetained(Boolean)
     * @see Item#isRetained()
     * @see #clearAll()
     * @see #addData(String, Func)
     */
    void clear();

    /**
     * Removes <em>all</em> items, lines, and actions from the receiver
     *
     * @see #clear()
     */
    void clearAll();

    /**
     * In addition to items and lines, a telemetry may also contain a list of actions.
     * When the telemetry is to be updated, these actions are evaluated before the telemetry
     * lines are composed just prior to transmission. A typical use of such actions is to
     * initialize some state variable, parts of which are subsequently displayed in items.
     * This can help avoid needless re-evaluation.
     *
     * <p>Actions are cleared with {@link #clearAll()}, and can be removed with {@link
     * #removeAction(Object) removeAction()}.</p>
     *
     * @param action    the action to execute before composing the lines telemetry
     * @return          a token by which the action can be later removed.
     * @see #addData(String, Object)
     * @see #removeAction(Object)
     * @see #addLine()
     * @see #update()
     */
    Object addAction(Runnable action);

    /**
     * Removes a previously added action from the receiver.
     * @param token the token previously returned from {@link #addAction(Runnable) addAction()}.
     * @return whether any change was made to the receiver
     */
    boolean removeAction(Object token);

    //----------------------------------------------------------------------------------------------
    // Text to Speech
    //----------------------------------------------------------------------------------------------

    /**
     * Directs the Driver Station device to speak the given text using TextToSpeech functionality,
     * with the same language and country codes that were previously used, or the default language
     * and country.
     *
     * @param text the text to be spoken
     */
    void speak(String text);

    /**
     * Directs the Driver Station device to speak the given text using TextToSpeech functionality,
     * with the given language and country codes.
     *
     * @param text          the text to be spoken
     * @param languageCode  an ISO 639 alpha-2 or alpha-3 language code, or a language subtag up to
     *                      8 characters in length
     * @param countryCode   an ISO 3166 alpha-2 country code, or a UN M.49 numeric-3 area code
     */
    void speak(String text, String languageCode, String countryCode);

    //----------------------------------------------------------------------------------------------
    // Transmission
    //----------------------------------------------------------------------------------------------

    /**
     * Sends the receiver {@link Telemetry} to the driver station if more than the {@link #getMsTransmissionInterval()
     * transmission interval} has elapsed since the last transmission, or schedules the transmission
     * of the receiver should no subsequent {@link Telemetry} state be scheduled for transmission before
     * the {@link #getMsTransmissionInterval() transmission interval} expires.
     * @return whether a transmission to the driver station occurred or not
     */
    boolean update();

    //----------------------------------------------------------------------------------------------
    // Data Lines
    //----------------------------------------------------------------------------------------------

    /**
     * Instances of {@link Line} build lines of data on the driver station telemetry display.
     */
    interface Line
    {
        /**
         * Adds a new data item at the end of the line which is the receiver.
         * @see Telemetry#addData(String, String, Object...)
         */
        Item addData(String caption, String format, Object... args);
        /**
         * Adds a new data item at the end of the line which is the receiver.
         * @see Telemetry#addData(String, Object)
         */
        Item addData(String caption, Object value);
        /**
         * Adds a new data item at the end of the line which is the receiver.
         * @see Telemetry#addData(String, Func)
         */
        <T> Item addData(String caption, Func<T> valueProducer);
        /**
         * Adds a new data item at the end of the line which is the receiver.
         * @see Telemetry#addData(String, String, Func)
         */
        <T> Item addData(String caption, String format, Func<T> valueProducer);
    }

    /**
     * Creates and returns a new line in the receiver {@link Telemetry}.
     * @return a new line in the receiver {@link Telemetry}
     */
    Line addLine();

    /**
     * Creates and returns a new line in the receiver {@link Telemetry}.
     * @param lineCaption the caption for the line
     * @return a new line in the receiver {@link Telemetry}
     */
    Line addLine(String lineCaption);

    /**
     * Removes a line from the receiver telemetry, if present.
     * @param line the line to be removed
     * @return whether any change was made to the receiver
     */
    boolean removeLine(Line line);

    //----------------------------------------------------------------------------------------------
    // Data Items
    //----------------------------------------------------------------------------------------------

    /**
     * Instances of {@link Item} represent an item of data on the drive station telemetry display.
     *
     * @see #addData(String, Object)
     */
    interface Item
    {
        /**
         * Returns the caption associated with this item.
         * @return the caption associated with this item.
         * @see #setCaption(String)
         * @see #addData(String, Object)
         */
        String getCaption();

        /**
         * Sets the caption associated with this item.
         * @param caption the new caption associated with this item.
         * @return the receiver
         * @see #getCaption()
         */
        Item setCaption(String caption);

        /**
         * Updates the value of this item to be the result of the indicated string formatting operation.
         * @param format    the format of the data
         * @param args      the arguments associated with the format
         * @return the receiver
         * @see #addData(String, String, Object...)
         */
        Item setValue(String format, Object...args);

        /**
         * Updates the value of this item to be the result of applying {@link Object#toString()}
         * to the indicated object.
         * @param value the object to which {@link Object#toString()} should be applied
         * @return the receiver
         * @see #addData(String, Object)
         */
        Item setValue(Object value);

        /**
         * Updates the value of this item to be the indicated value producer.
         * @param valueProducer an object that produces values to be rendered.
         * @return the receiver
         * @see #addData(String, Func)
         */
        <T> Item setValue(Func<T> valueProducer);

        /**
         * Updates the value of this item to be the indicated value producer.
         * @param format        this string used to format values produced
         * @param valueProducer an object that produces values to be rendered.
         * @return the receiver
         * @see #addData(String, String, Func)
         */
        <T> Item setValue(String format, Func<T> valueProducer);

        /**
         * Sets whether the item is to be retained in clear() operation or not.
         * This is initially true for items that whose value is computed with a
         * value producer; otherwise, it is initially false.
         * @param retained if true, then the value will be retained during a clear(). Null will
         *                 return the setting to its initial value.
         * @return the receiver
         * @see #clear()
         * @see #isRetained()
         */
        Item setRetained(@Nullable Boolean retained);

        /**
         * Returns whether the item is to be retained in a clear() operation.
         * @return whether the item is to be retained in a clear() operation.
         * @see #setRetained(Boolean)
         */
        boolean isRetained();

        /**
         * Adds a new data item in the associated {@link Telemetry} immediately following the receiver.
         * @see #addData(String, String, Object...)
         */
        Item addData(String caption, String format, Object... args);

        /**
         * Adds a new data item in the associated {@link Telemetry} immediately following the receiver.
         * @see #addData(String, Object)
         */
        Item addData(String caption, Object value);

        /**
         * Adds a new data item in the associated {@link Telemetry} immediately following the receiver.
         * @see #addData(String, Func)
         */
        <T> Item addData(String caption, Func<T> valueProducer);

        /**
         * Adds a new data item in the associated {@link Telemetry} immediately following the receiver.
         * @see #addData(String, String, Func)
         */
        <T> Item addData(String caption, String format, Func<T> valueProducer);
    }

    //----------------------------------------------------------------------------------------------
    // Properties
    //----------------------------------------------------------------------------------------------

    /**
     * Answers whether {@link #clear()} is automatically called after each call to {@link #update()}.
     * @return whether {@link #clear()} is automatically called after each call to {@link #update()}.
     * @see #setAutoClear(boolean)
     */
    boolean isAutoClear();

    /**
     * Sets whether {@link #clear()} is automatically called after each call to {@link #update()}.
     * @param autoClear if true, {@link #clear()} is automatically called after each call to {@link #update()}.
     */
    void setAutoClear(boolean autoClear);

    /**
     * Returns the minimum interval between {@link Telemetry} transmissions from the robot controller
     * to the driver station
     * @return the minimum interval between {@link Telemetry} transmissions from the robot controller to the diver station
     * @see #setMsTransmissionInterval(int)
     */
    int getMsTransmissionInterval();

    /**
     * Sets the minimum interval between {@link Telemetry} transmissions from the robot controller
     * to the driver station.
     * @param msTransmissionInterval  the minimum interval between {@link Telemetry} transmissions
     *                                from the robot controller to the driver station
     * @see #getMsTransmissionInterval()
     */
    void setMsTransmissionInterval(int msTransmissionInterval);

    /**
     * Returns the string which is used to separate {@link Item}s contained within a line. The default
     * separator is " | ".
     * @return the string which is use to separate {@link Item}s contained within a line.
     * @see #setItemSeparator(String)
     * @see #addLine()
     */
    String getItemSeparator();

    /**
     * @see #setItemSeparator(String)
     */
    void setItemSeparator(String itemSeparator);

    /**
     * Returns the string which is used to separate caption from value within a {@link Telemetry}
     * {@link Item}. The default separator is " : ";
     * @return the string which is used to separate caption from value within a {@link Telemetry} {@link Item}.
     */
    String getCaptionValueSeparator();

    /**
     * @see #getCaptionValueSeparator()
     */
    void setCaptionValueSeparator(String captionValueSeparator);

    enum DisplayFormat
    {
        CLASSIC,   // What you've all come to know and love (or not) since 2015
        MONOSPACE, // Same as classic, except uses a monospaced font so you can column align data
        HTML;      // Allows use of a subset of HTML tags, enabling "rich text" display (e.g. color & size)
    }

    /**
     * Sets the telemetry display format on the Driver Station. See the comments on {@link DisplayFormat}.
     *
     * @param displayFormat the telemetry display format the Driver Station should use
     */
    void setDisplayFormat(DisplayFormat displayFormat);

    //----------------------------------------------------------------------------------------------
    // Properties
    //----------------------------------------------------------------------------------------------

    /**
     * The {@link Log} in a {@link Telemetry} instance provides an append-only list of messages
     * that appear on the driver station below the {@link Item}s of the {@link Telemetry}.
     * @see #log()
     * @see #addData(String, Object)
     */
    interface Log
    {
        /**
         * {@link DisplayOrder} instances indicate the desired ordering of a {@link #log()}.
         */
        enum DisplayOrder { NEWEST_FIRST, OLDEST_FIRST }

        /**
         * Returns the maximum number of lines which will be retained in a {@link #log()} and
         * shown on the driver station display.
         * @return the maximum number of lines which will be retained in a {@link #log()}
         * @see #setCapacity(int)
         */
        int getCapacity();

        /**
         * @see #getCapacity()
         */
        void setCapacity(int capacity);

        /**
         * Returns the order in which data in log is to be displayed on the driver station.
         * @return the order in which data in log is to be displayed on the driver station.
         * @see #setDisplayOrder(DisplayOrder)
         */
        DisplayOrder getDisplayOrder();

        /**
         * @see #getDisplayOrder()
         */
        void setDisplayOrder(DisplayOrder displayOrder);

        /**
         * Adds a new entry the the log. Transmits the updated log to the driver station at the
         * earliest opportunity.
         * @param entry     the new log entry to add
         */
        void add(String entry);

        /**
         * Adds a new entry to the log. Transmits the updated log to the driver station at the
         * earliest opportunity.
         * @param format    the format string used to format the log entry
         * @param args      the data used to format the log entry
         */
        void add(String format, Object...args);

        /**
         * Removes all entries from this {@link Log}
         */
        void clear();
    }

    /**
     * Returns the log of this {@link Telemetry} to which log entries may be appended.
     * @return the log of this {@link Telemetry} to which log entries may be appended.
     * @see Log#addData(String, Object)
     */
    Log log();
}
