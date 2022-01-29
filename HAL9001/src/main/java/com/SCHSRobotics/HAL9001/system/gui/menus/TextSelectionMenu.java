package com.SCHSRobotics.HAL9001.system.gui.menus;

import static java.lang.Math.min;

import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.SelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.TextElement;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton;
import com.SCHSRobotics.HAL9001.util.constant.Charset;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALGUIException;
import com.SCHSRobotics.HAL9001.util.misc.StringUtils;
import com.SCHSRobotics.HAL9001.util.misc.UniqueID;

import org.jetbrains.annotations.NotNull;

/**
 * A HAL menu used for entering text via the gamepad as a form of user input.
 * <p>
 * Creation Date: 9/10/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see UniqueID
 * @see TextElement
 * @see ViewButton
 * @see EntireViewButton
 * @see Payload
 * @see Charset
 * @see SelectionZone
 * @since 1.1.0
 */
public class TextSelectionMenu extends HALMenu {

    /*
            MENU FORMAT:
            ###################################

            ____________________ (_ = spaces) 0
            #|abc #|def #|ghi                 1
            #|jkl #|mno #|pqr                 2
            #|stu #|vwx #|yz0                 3
            #|123 #|456 #|789                 4
            #|!?@ #|$%^ #|&*                  5
            #|<--       -->|#                 6
            #|Done                            7
    */

    /**
     * Unique ids for all the options that can be provided to the menu via the payload. The only exception is ENTERED_TEXT_ID, which is the ID used to retrieve the text entered in the menu from the output payload.
     */
    public static final UniqueID ENTERED_TEXT_ID = new UniqueID("output text"),
            CHAR_SET_ID = new UniqueID("charset"),
            NEXT_MENU_ID = new UniqueID("next menu"),
            BACK_BUTTON_ID = new UniqueID("back button"),
            FORWARD_BUTTON_ID = new UniqueID("forward button"),
            LEFT_BUTTON_ID = new UniqueID("left button"),
            RIGHT_BUTTON_ID = new UniqueID("right button");
    //The prefix for each group of letters.
    private static final String SELECTION_PREFIX = "#|";
    //Special characters used to format the menu.
    private static final char UNDERLINE_CHAR = '\u0332', SPACE_CHAR = '_';
    //Constants that define the dimensions of the list.
    private static final int MAX_CHAR_CHUNK_SIZE = 3, MAX_CHUNKS_PER_LINE = 3, MAX_ROW_LENGTH_CHARS = (MAX_CHAR_CHUNK_SIZE * MAX_CHUNKS_PER_LINE) + (MAX_CHUNKS_PER_LINE - 1) + (MAX_CHUNKS_PER_LINE * SELECTION_PREFIX.length());
    //Constant selection zone regions for each row and the arrows at the bottom of the screen.
    private static final boolean[] ROW_SELECTION_ZONE = new boolean[MAX_ROW_LENGTH_CHARS], ARROW_SELECTION_ZONE = new boolean[MAX_ROW_LENGTH_CHARS];

    //Initializes the constant selection zones.
    static {
        ARROW_SELECTION_ZONE[0] = true;
        ARROW_SELECTION_ZONE[MAX_ROW_LENGTH_CHARS - 1] = true;

        for (int i = 0; i < MAX_ROW_LENGTH_CHARS; i += MAX_CHAR_CHUNK_SIZE + SELECTION_PREFIX.length() + 1) {
            ROW_SELECTION_ZONE[i] = true;
        }
    }

    //The text element that is used to display entered text.
    private TextElement inputTextElement = new TextElement("" + SPACE_CHAR);
    //The class of the next menu that will be opened after the text selection process has completed.
    private Class<HALMenu> nextMenu;
    //A string containing all valid characters that can be entered in the menu.
    private String validChars;

    /**
     * The constructor for TextSelectionMenu. Sets the first row in the selection zone to false so the cursor cannot travel there.
     *
     * @param payload The payload passed to this menu.
     *
     * @see HALMenu
     * @see Payload
     * @see SelectionZone
     */
    public TextSelectionMenu(Payload payload) {
        super(payload);
        selectionZone = new SelectionZone(new boolean[][]{{false}});
    }

    /**
     * The constructor for TextSelectionMenu. This constructor just creates an empty payload.
     *
     * @see HALMenu
     * @see Payload
     * @see SelectionZone
     */
    public TextSelectionMenu() {
        this(new Payload());
    }

    /**
     * Formats the provided string so that the char at charIdx appears to be selected.
     *
     * @param input The input string of entered text.
     * @param charIdx The index of the char to make selected.
     * @return A formatted string where the char at charIdx appears to be selected.
     *
     * @see StringUtils
     */
    @NotNull
    private static String selectChar(@NotNull String input, int charIdx) {
        char selectedChar = input.charAt(charIdx);

        if (selectedChar == SPACE_CHAR) return input.replaceAll("" + UNDERLINE_CHAR, "");
        else if (selectedChar == ' ')
            return StringUtils.setChar(input, charIdx, '_').replaceAll("" + UNDERLINE_CHAR, "");
        else {
            String nonUnderlineText = input.replaceAll("" + UNDERLINE_CHAR, "");

            String firstHalf = nonUnderlineText.substring(0, charIdx + 1);
            String secondHalf = nonUnderlineText.substring(charIdx + 1);
            return firstHalf + UNDERLINE_CHAR + secondHalf;
        }
    }

    /**
     * Gets the index of the currently selected character in the provided string.
     *
     * @param inputText The formatted input string with a single selected character.
     * @return The index of the selected character within the given formatted string.
     * @throws HALGUIException Throws this exception when the provided string does not have any "indicator characters" that denote the position of the selected character.
     */
    private static int getCurrentSelectedCharIdx(@NotNull String inputText) {
        int selectedIdx = inputText.indexOf(UNDERLINE_CHAR) - 1;
        if (selectedIdx == -2) selectedIdx = inputText.indexOf('_');
        ExceptionChecker.assertFalse(selectedIdx == -1, new HALGUIException("No indicator characters found in input string, we lost your place in the input string somehow :("));
        return selectedIdx;
    }

    /**
     * The TextSelectionMenu's init function.
     *
     * @param payload The payload passed to this menu.
     *
     * @see HALMenu
     * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
     * @see Payload
     * @see ViewButton
     * @see EntireViewButton
     */
    @Override
    protected void init(@NotNull Payload payload) {
        if (payload.idPresent(CHAR_SET_ID)) {
            Object obj = payload.get(CHAR_SET_ID);
            if (obj instanceof Charset) validChars = ((Charset) obj).getChars();
            else if (obj instanceof String) validChars = (String) obj;
        } else validChars = Charset.ALPHANUMERIC.getChars();

        nextMenu = payload.idPresent(NEXT_MENU_ID) ? payload.get(NEXT_MENU_ID) : null;

        //Add a left button to move the selection cursor left. Only happens if a left button is specified in the payload.
        if (payload.idPresent(LEFT_BUTTON_ID)) {
            addItem(new EntireViewButton()
                    .onClick(payload.get(LEFT_BUTTON_ID), (DataPacket packet) -> {
                        String originalText = inputTextElement.getText();

                        int currentlySelectedCharIdx = getCurrentSelectedCharIdx(originalText);
                        char selectedChar = originalText.charAt(currentlySelectedCharIdx);

                        if (selectedChar == SPACE_CHAR)
                            originalText = StringUtils.setChar(originalText, currentlySelectedCharIdx, ' ');

                        String nonUnderlined = originalText.replaceAll(""+UNDERLINE_CHAR, "");

                        if(currentlySelectedCharIdx > 0) {
                            if (currentlySelectedCharIdx == nonUnderlined.length() - 1 && selectedChar == SPACE_CHAR) {
                                nonUnderlined = StringUtils.removeLastChar(nonUnderlined);
                            }
                            inputTextElement.setText(selectChar(nonUnderlined, currentlySelectedCharIdx - 1));
                        }
                    }));
        }
        //Add a right button to move the selection cursor left. Only happens if a right button is specified in the payload.
        if(payload.idPresent(RIGHT_BUTTON_ID)) {
            addItem(new EntireViewButton()
                    .onClick(payload.get(RIGHT_BUTTON_ID), (DataPacket packet) -> {
                        String originalText = inputTextElement.getText();

                        int currentlySelectedCharIdx = getCurrentSelectedCharIdx(originalText);
                        char selectedChar = originalText.charAt(currentlySelectedCharIdx);

                        if (selectedChar == SPACE_CHAR)
                            originalText = StringUtils.setChar(originalText, currentlySelectedCharIdx, ' ');

                        String nonUnderlined = originalText.replaceAll("" + UNDERLINE_CHAR, "");

                        if (currentlySelectedCharIdx + 1 == nonUnderlined.length())
                            nonUnderlined += ' ';
                        inputTextElement.setText(selectChar(nonUnderlined, currentlySelectedCharIdx + 1));
                    }));
        }
        //Add a back button to go back one menu. Only happens if a back button is specified in the payload.

        if (payload.idPresent(BACK_BUTTON_ID)) {
            addItem(new EntireViewButton()
                    .onClick(payload.get(BACK_BUTTON_ID), (DataPacket packet) -> gui.back(payload)));
        }
        //Add a back button to go forward one menu. Only happens if a forward button is specified in the payload.
        if (payload.idPresent(FORWARD_BUTTON_ID)) {
            addItem(new EntireViewButton()
                    .onClick(payload.get(FORWARD_BUTTON_ID), (DataPacket packet) -> {
                        String parsedText = inputTextElement.getText().replaceAll("[" + UNDERLINE_CHAR + SPACE_CHAR + "]", "");
                        payload.add(ENTERED_TEXT_ID, StringUtils.bilateralStrip(parsedText, ' '));
                        gui.forward(payload);
                    }));
        }

        addItem(inputTextElement);

        //Splits valid entry characters into chunks of a specific size, then adds them all as selectable options.
        int chunkIdxInRow = 0;
        StringBuilder rowText = new StringBuilder();
        String[] characterChunks = StringUtils.splitEqually(validChars, MAX_CHAR_CHUNK_SIZE);
        for(String chunk : characterChunks) {
            chunkIdxInRow++;

            rowText.append(SELECTION_PREFIX);
            rowText.append(chunk);
            rowText.append(' ');

            if(chunkIdxInRow % MAX_CHUNKS_PER_LINE == 0) {
                addItem(new ViewButton(rowText.toString())
                        .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> {
                            ViewButton thisButton = packet.getListener();
                            String thisText = thisButton.getText();

                            String originalText = inputTextElement.getText();

                            int chunkStart = getCursorX() + SELECTION_PREFIX.length();
                            int chunkEnd = chunkStart + min(thisText.length() - chunkStart, MAX_CHAR_CHUNK_SIZE);
                            String characterOptions = thisText.substring(chunkStart, chunkEnd).trim() + SPACE_CHAR;

                            int currentlySelectedCharIdx = getCurrentSelectedCharIdx(originalText);
                            int currentOptionIdx = characterOptions.indexOf(originalText.charAt(currentlySelectedCharIdx));

                            //Note: if idx is -1 (it didn't find the char), this will make it the first char in the options.;
                            int nextOptionIdx = (currentOptionIdx + 1) % characterOptions.length();
                            char nextOption = characterOptions.charAt(nextOptionIdx);

                            inputTextElement.setText(selectChar(StringUtils.setChar(originalText, currentlySelectedCharIdx, nextOption), currentlySelectedCharIdx));
                        }));
                selectionZone.addRow(ROW_SELECTION_ZONE);
                rowText.delete(0, rowText.length());
                chunkIdxInRow = 0;
            }
        }
        //If it didn't finish the last line, add the rest of the row text to the screen.
        //todo check if this part got finished.
        if (chunkIdxInRow != 0) {
            addItem(new TextElement(rowText.toString()));
            selectionZone.addRow(ROW_SELECTION_ZONE);
        }

        //Adds the arrow selection region and the done selection region.
        selectionZone.addRow(ARROW_SELECTION_ZONE);
        selectionZone.addRow(new boolean[]{true});

        //Adds the left and right arrow buttons.
        addItem(new ViewButton(SELECTION_PREFIX+"<--       -->"+StringUtils.reverseString(SELECTION_PREFIX))
                .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> {
                    String originalText = inputTextElement.getText();

                    int currentlySelectedCharIdx = getCurrentSelectedCharIdx(originalText);
                    char selectedChar = originalText.charAt(currentlySelectedCharIdx);

                    if (selectedChar == SPACE_CHAR)
                        originalText = StringUtils.setChar(originalText, currentlySelectedCharIdx, ' ');

                    String nonUnderlined = originalText.replaceAll(""+UNDERLINE_CHAR, "");

                    //Cursor X refers to user's cursor position, NOT the selected character.
                    if(getCursorX() == 0 && currentlySelectedCharIdx > 0) {
                        if (currentlySelectedCharIdx == nonUnderlined.length() - 1 && selectedChar == SPACE_CHAR) {
                            nonUnderlined = StringUtils.removeLastChar(nonUnderlined);
                        }
                        inputTextElement.setText(selectChar(nonUnderlined, currentlySelectedCharIdx - 1));
                    } else if (getCursorX() == MAX_ROW_LENGTH_CHARS - 1) {
                        if (currentlySelectedCharIdx + 1 == nonUnderlined.length())
                            nonUnderlined += ' ';
                        inputTextElement.setText(selectChar(nonUnderlined, currentlySelectedCharIdx + 1));
                    }
                }));

        //Adds the done button.
        addItem(new ViewButton(SELECTION_PREFIX + "Done")
                .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> {
                    String parsedText = inputTextElement.getText().replaceAll("[" + UNDERLINE_CHAR + SPACE_CHAR + "]", "");
                    payload.add(ENTERED_TEXT_ID, StringUtils.bilateralStrip(parsedText, ' '));
                    if (nextMenu == null) gui.back(payload);
                    else gui.inflate(nextMenu, payload);
                }));

        //Sets the cursor to second line so it doesn't start in the prohibited zone.
        setCursorPos(0, 1);
    }
}
