// ignores most of the code
// cSpell:ignoreRegExp /(^(?!\s*(\/\/)|(\/\*)).*[;:)}=,{])/gm

// ignores any word in quotes
// cSpell:ignoreRegExp /\"\S*\"/g

//--- ignores HEX literals
// cSpell:ignoreRegExp /0x[A-Z]+/g

//--- ignores any preprocessor directive (i.e #define)
// cSpell:ignoreRegExp /(^#.*)/gm

/// words to ignore
// cSpell:ignore pico PSRAM btn btns spec'd dbgserPrintln dbgser Println

/// spell check extension defaults to checking each part of camel case words as separate words.

/*

Commands:
How to set up LCD's from i2c master:
1. Set columns and rows - 4 bytes: 0x05, LCD_ID, cols, rows
2. Start LCD - 2 bytes: 0x0b, LCD_ID

Create Custom Chars
1. Send for each character - 9 bytes: 0x2$ (where $ identifies character number 0 thru 7), character data for 8 bytes
2. Send character data to LCD - 2 byte: 0x30, bit mask of LCD_IDs to send it to
      -This will send whatever is in the temporary custom character buffer.

Display Text
1. 2 + any number of bytes up to 30(buffer max of 32 bytes): 0xa$ ($ = LCD_ID), 0b $$@@ @@@@@ ($$ = row, @@@@@@ = col), text

*/

// #include <Arduino.h>
/* #region DEFINEs and debugging */
#define DEBUG
#define ESP_PROGRAMMING_MODEn
#define DEBUG_LEVEL 5        // 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
#define NUMBER_OF_SCENES 200 // 1500 max. Larger values make booting slower. Consider keeping this at ~500 or less.
/* #endregion */

#include "include/midiFt_lib.cpp"
#include <Wire.h>
#include <MIDI.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <cstdint>
// #include <NativeEthernet.h>

void test(){
    byte thing = 0;
}
/* #region Global Declarations */

// The next 4 lines use a #define macro that expands to setup the instance.
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI1);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI2);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, MIDI3);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial7, MIDI4);

// This does the same thing as the macro, but stores the instances in an array
// that is easier to programmatically use.
// Uses exactly the same amount of RAM as the 4 macro calls, but is easier to use.
midi::SerialMIDI<HardwareSerial> serialMIDI1(Serial1);
midi::SerialMIDI<HardwareSerial> serialMIDI2(Serial2);
midi::SerialMIDI<HardwareSerial> serialMIDI3(Serial6);
midi::SerialMIDI<HardwareSerial> serialMIDI4(Serial7);
midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> HW_midi[] = {
        (midi::SerialMIDI<HardwareSerial> &)serialMIDI1,
        (midi::SerialMIDI<HardwareSerial> &)serialMIDI2,
        (midi::SerialMIDI<HardwareSerial> &)serialMIDI3,
        (midi::SerialMIDI<HardwareSerial> &)serialMIDI4 };

/// \brief A structured packet of data containing a 2 byte start sequence, a 2 byte command, 32 bytes of data, and a 32bit CRC.
/// \param : access any byte of the packet with ESP_SerPacket[].
SerialPacket ESP_SerPacket = SerialPacket();
/// \brief A structured packet of data containing a 1 byte command, 4 bytes of data, and a 8bit CRC.
SerialMessage ESP_ShortMessage = SerialMessage();
///\brief File object for reading files from SD card
File sdCardFile;
bool sdCardInit = false;
// bool ethernetLinkUp = false;
// unsigned long ethernetLinkTimeout = 5000;     // milliseconds
// unsigned long ethernetResponseTimeout = 5000; // milliseconds
///\brief All of the user configurable settings.
PrefsObj preferences = PrefsObj();
///\brief Object to hold all scene data. Is stored on PSRAM.
EXTMEM SceneObj midiFT = SceneObj(NUMBER_OF_SCENES);
///\brief Object to hold currently selected scene. Stored in RAM for (slightly) faster access.
SceneObjSingle currentSceneInRAM = SceneObjSingle();
///\brief Microsecond timers used in loop()
elapsedMicros elapsedTimer1, elapsedTimer2, elapsedTimer3, elapsedTimer4;
///\brief Buffer for i2c commands to LCD controllers
WireBuffer wireBufForLCDs = WireBuffer();
///\brief Buffer for i2c commands to RasPi Pico
// WireBuffer wireBufForPICO = WireBuffer();
///\brief Lookup table for button pin numbers and button de-bouncing objects.
const uint8_t BtnPins[] = { 13, 41, 40, 39, 38, 37, 36, 33, 23, 22 };
Bounce BtnDebouncer[10] = { Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce() };
///\brief The currently selected scene.
uint16_t currentScene = 0;
// Values to keep track of the current position of the "top row text" of each button.
uint8_t topRowTextPositions[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// Variables for freeram()
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char* __brkval;
///\brief FIFO for queuing actions to prevent blocking and allow "wait" actions.
///mainActionQ for mainBtns, and extActionQ for extBtns. One queue for each button.
void setCurrentScene(int sceneNumber);
buttonActionQueue mainActionQ[] = {
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(true,midiFT,HW_midi,setCurrentScene,preferences)
};
buttonActionQueue extActionQ[] = {
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences),
    buttonActionQueue(false,midiFT,HW_midi,setCurrentScene,preferences)
};
void extActionsQueueHandler(uint8_t btnNum, uint8_t scene, uint16_t val);
RasPiPico picoOBJ = RasPiPico((uint8_t)ADDR_I2C_TO_EXT_BTN_CONTROLLER, preferences, extActionsQueueHandler);

// unsigned long loopCounter = 0;
// unsigned long loopCounterMark = 0;
// unsigned long loopCounterMin = 0xffff;
// unsigned long loopCounterMax = 0;
/* #endregion Global Declarations */


/* #region function definitions */
void setLCDbrightness(byte pwm_val);
void LCDs_sendCustChar(byte lcdIdBitmask, byte charNum, byte dataArr[]);
bool sendDataToESP(uint16_t command, uint8_t dataArr[], uint8_t len, bool isFullPacket);
bool sendDataToESP(uint16_t command, uint8_t dataArr[], bool isFullPacket);
bool sendDataToESP(uint16_t command, uint8_t data, bool isFullPacket);
void LCDs_printCustomChar(byte lcdIdBitmask, byte charNum, int col, int row);
void setupLCDs();
void waitForLCDready();
bool isLCDready();
void clearScene(uint16_t num);
void loadSceneDataIntoPSRAM(unsigned int sceneNumber, unsigned int dataType, byte dataArray[], unsigned int arrayLength);
bool loadPrefsFromFile();
void updateHardwareMIDIthru();
void LCDS_updateText();
void enter_ESP8266_ProgrammingMode();
void handleESP8266Serial();
void nop();
void setup();
int freeram();
void loop();
/* #endregion function definitions */


/// \brief Calculate amount of free RAM
/// \returns Integer representing number of available bytes.
int freeram() {
    return (char*)&_heap_end - __brkval;
}

/// \brief Handle button presses event
/// \param btnID Id 0-9 of pressed button
void buttonPressed(uint8_t btnID) {
    // debugserialPrint(4, "Button ");
    // debugserialPrint(4, btnID);
    // debugserialPrintln(4, " was pressed.");
    for (uint8_t i = 0;i < 32;i++) {
        mainActionQ[btnID].addAction(btnID, i, currentScene);
        if (midiFT.scenesArr[currentScene].mainButtons[btnID].Actions[i + 1].action == 0xff)i = 32;
    }
}

void extActionsQueueHandler(uint8_t btnNum, uint8_t scene, uint16_t val) {
    // btn numbers 0 to 7 are for TS/TRS button type pedals
    if (btnNum < 8) {
        for (uint8_t i = 0; i < 32; i++) {
            extActionQ[btnNum].addAction(btnNum, i, scene);
            if (midiFT.scenesArr[scene].extButtons[btnNum].Actions[i + 1].action == 0xff)i = 32;
        }
    } else {
        // btn numbers >=8 are for expression pedals.
        //@todo 
    }
}


/// \brief Do literally nothing. Exists for debugging purposes.
/// Compiler will almost certainly remove any call to this function.
void nop() {}

/// \brief Write the appropriate commands on i2c bus to set LCD brightness.
/// \param pwm_val Value that the LCD controller should write to the PWM.
void setLCDbrightness(byte pwm_val) {
    Wire.beginTransmission(ADDR_I2C_TO_LCD);
    Wire.write(LCD_COMMAND_SET_BL_BRIGHTNESS);
    Wire.write(pwm_val);
    Wire.endTransmission();
    waitForLCDready();
}

/// \brief Write appropriate commands on the i2c bus to save a custom character to the LCD's.
/// \param lcdIdBitMask Used to determine which LCD's to write the custom character to.
/// \param charNum Each LCD can store up to 8 custom characters.
/// \param dataArr Array of bytes with custom character data
void LCDs_sendCustChar(byte lcdIdBitmask, byte charNum, byte dataArr[]) {
    Wire.beginTransmission(ADDR_I2C_TO_LCD);
    Wire.write(LCD_COMMAND_MK_CUST_CHAR_BASE + charNum);
    for (int i = 0; i < 8; i++) {
        Wire.write(dataArr[i]);
    }
    Wire.endTransmission();
    waitForLCDready();
    Wire.beginTransmission(ADDR_I2C_TO_LCD);
    Wire.write(LCD_COMMAND_SEND_CUST_CHAR);
    Wire.write(lcdIdBitmask);
    Wire.endTransmission();
    waitForLCDready();
}

/// \brief Write appropriate commands to the i2c bus display a saved custom character on any or all LCD's.
/// Must call "LCDs_sendCustChar()" first in order to ensure valid data has been saved on the LCD's.
/// \param lcdIdBitmask Used to determine which LCD's should be told to display the custom character referenced by charNum.
/// When printing custom characters on multiple LCD's at once, note that each LCD may have different data.
/// \param col \param row Location on the LCD to print the character. No checking is performed to ensure the location is valid.
void LCDs_printCustomChar(byte lcdIdBitmask, byte charNum, int col, int row) {
    byte colRow = (row << 6) + col;
    for (int i = 0; i < 8; i++) {
        if (bitRead(lcdIdBitmask, i) == 1) {
            Wire.beginTransmission(ADDR_I2C_TO_LCD);
            Wire.write(LCD_COMMAND_DISP_TEXT_BASE + i);
            Wire.write(colRow);
            Wire.write(charNum);
            Wire.endTransmission();
            waitForLCDready();
        }
    }
}

/// \brief Must be called prior to any other LCD related function.
void setupLCDs() {
    for (int i = 0; i < 5; i++) {
        Wire.beginTransmission(ADDR_I2C_TO_LCD); // transmit to device #8
        Wire.write(LCD_COMMAND_SET_COLS_ROWS);   // set cols and rows
        Wire.write(i);
        Wire.write(20);
        Wire.write(4);
        Wire.endTransmission(); // stop transmitting
        waitForLCDready();
        Wire.beginTransmission(ADDR_I2C_TO_LCD); // transmit to device #8
        Wire.write(LCD_COMMAND_START_LCD);       // start lcd
        Wire.write(i);                           // lcd id
        Wire.endTransmission();                  // stop transmitting
        waitForLCDready();
    }
    byte up1[8] = { 0b00001, 0b00011, 0b00011, 0b00111, 0b00111, 0b01111, 0b01111, 0b11111 };
    byte up2[8] = { 0b10000, 0b11000, 0b11000, 0b11100, 0b11100, 0b11110, 0b11110, 0b11111 };
    byte up3[8] = { 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00000, 0b00000 };
    byte up4[8] = { 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b00000, 0b00000 };
    byte down1[8] = { 0b11111, 0b01111, 0b01111, 0b00111, 0b00111, 0b00011, 0b00011, 0b00001 };
    byte down2[8] = { 0b11111, 0b11110, 0b11110, 0b11100, 0b11100, 0b11000, 0b11000, 0b10000 };
    byte down3[8] = { 0b00000, 0b00000, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011 };
    byte down4[8] = { 0b00000, 0b00000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000 };
    LCDs_sendCustChar(0b00011111, 0, up1);
    LCDs_sendCustChar(0b00011111, 1, up2);
    LCDs_sendCustChar(0b00011111, 2, up3);
    LCDs_sendCustChar(0b00011111, 3, up4);
    LCDs_sendCustChar(0b00011111, 6, down1);
    LCDs_sendCustChar(0b00011111, 7, down2);
    LCDs_sendCustChar(0b00011111, 4, down3);
    LCDs_sendCustChar(0b00011111, 5, down4);
    const uint8_t arrowLocation[4][8] = {
        {18, 19, 18, 19, 18, 19, 18, 19},
        {0, 1, 0, 1, 18, 19, 18, 19},
        {18, 19, 18, 19, 0, 1, 0, 1},
        {0, 1, 0, 1, 0, 1, 0, 1} };
    for (uint8_t i = 0; i < 8; i++)
        LCDs_printCustomChar(0b00011111, i, arrowLocation[preferences.arrowPos][i], i >> 1);
}

///\brief Sends commands over i2c to print text on the LCD's.
///\param lcdID Id of the LCD to print to.
///\param str Char array to print.
///\param len Length of str[]
///\param col \param row Location on LCD screen for first character of str[]. Both col and row are 0 indexed.
///\param useBigBuffer If true, i2c will be handeld by the WireBuffer::wireBuf object functions. Allows commands to be stacked
/// faster than the Wire library can send them. Must call WireBuffer::sendNextOutMessage() very often. Default is false.
///\param wait If true, function will not return until the LCD controller is ready for more data. Has no effect is useBigBuffer is true. Default is true.
void LCDs_printText(int lcdID, char str[], int len, int col, int row, bool useBigBuffer = false, bool wait = true) {
    if (useBigBuffer) {
        byte bytesToSend[len + 2];
        bytesToSend[0] = LCD_COMMAND_DISP_TEXT_BASE + lcdID;
        bytesToSend[1] = byte((row << 6) + col);
        for (uint16_t i = 2, p = 0; i < len + 2; i++)
            bytesToSend[i] = str[p++];
        wireBufForLCDs.addEntryToBuffer(ADDR_I2C_TO_LCD, bytesToSend, len + 2, 282 * (len * 2)); // wait time calculated to be about 0.5ms for each byte of data to transfer.
    } else {
        Wire.beginTransmission(ADDR_I2C_TO_LCD);
        Wire.write(LCD_COMMAND_DISP_TEXT_BASE + lcdID);
        Wire.write(byte((row << 6) + col));
        for (int i = 0; i < len; i++)
            Wire.write(byte(str[i]));
        Wire.endTransmission();
        if (wait)
            waitForLCDready();
    }
}

///\brief Read status from LCD controller and wait if not ready. Blocks code execution until LCD controller is ready for more data.
///This function is necessary to prevent interrupting the LCD controller while it is writing data to theLCD's.
///Must be called after every endTransmission when not using "WireBuffer::wireBufForLCDs".
void waitForLCDready() {
    bool isReady = false;
    while (!isReady) {
        Wire.requestFrom(ADDR_I2C_TO_LCD, 4);
        byte readBytes[4] = { 0, 0, 0, 0xaa };
        int byteNum = 0;
        while (Wire.available()) {
            readBytes[byteNum] = Wire.read();
            byteNum++;
        }
        isReady = readBytes[3] == 0xff;
    }
}

///\brief Determine if LCD controller is ready for more data.
///\return True is ready, false if not...
bool isLCDready() {
    Wire.requestFrom(ADDR_I2C_TO_LCD, 4);
    byte readBytes[4] = { 0, 0, 0, 0xaa };
    int byteNum = 0;
    while (Wire.available()) {
        readBytes[byteNum] = Wire.read();
        byteNum++;
    }
    return readBytes[3] == 0xff;
}

///\brief Reset all data in scene "num" to init values. This happens the object in RAM as well as PSRAM.
///\param num Scene number to be reset.
void clearScene(uint16_t num) {
    midiFT.scenesArr[num].resetToDefaults();
    setCurrentScene(num);
}

///\brief Load all scenes from files. This function expects there to be all least "NUMBER_OF_SCENES" scene files on the SD.
/// If not, it will return false and log the number of errors to the debug serial port, if enabled.
///\param sceneNumFileToLoad If specified, func will load specified scene. If not specified, will load all scenes from files.
///\return Boolean, true if all scenes were loaded, false if not.
bool loadScenesFromFile(int sceneFileNumToLoad = (NUMBER_OF_SCENES + 1)) {
    String fileName = "SCN";  // file name prefix
    String fileExt = ".TXT";  // file name suffix
    String fullFileName = ""; // a String to hold the concatenated filename
    char nameChars[15];
    int err = 0;


    // if sceneNumFileToLoad was not specified, set iteratorMax and sceneNumfileToLoad so that the iterator loop
    // loads all the file. Otherwise, sceneNumber will be sceneNumFileToLoad and iteratorMax is that + 1 (run loop once).
    unsigned int iteratorMax = 0;
    if (sceneFileNumToLoad > NUMBER_OF_SCENES) {
        iteratorMax = NUMBER_OF_SCENES;
        sceneFileNumToLoad = 0;
    } else {
        iteratorMax = sceneFileNumToLoad + 1;
    }


    // iterate through all the scenes
    for (unsigned int sceneNumber = sceneFileNumToLoad; sceneNumber < iteratorMax; sceneNumber++) {
        if (sceneNumber & 32) {
            char someChars[] = "#";
            LCDs_printText(0, someChars, 1, 16, 0);
        } else if ((sceneNumber + 16) & 32) {
            char someChars[] = " ";
            LCDs_printText(0, someChars, 1, 16, 0);
        }
        // load scene number "sceneNumber" from file
        fullFileName = fileName + sceneNumber + fileExt; // concatenate stuff to make the filename
        fullFileName.toCharArray(nameChars, 15);
        sdCardFile = SD.open(nameChars);
        if (sdCardFile) {
            while (sdCardFile.available()) {
                char readChar = sdCardFile.read();
                while (sdCardFile.available() && (readChar == '\n' || readChar == 0x0d || readChar == ' ' || readChar == '\t' || readChar == '\r')) {
                    readChar = sdCardFile.read(); // read through white space if present
                }
                char asciiCodedHex[5] = "    "; // null terminated char array of 4 spaces
                unsigned int ind = 0;           // index to keep track of how much useful data is in the array
                unsigned int readInt = 0;       // integer version of the asciiCodedHex
                byte dataArray[256];            // an array to accumulate data.
                // @TODO pare down the size of dataArray
                for (int i = 0; i < 256; i++)
                    dataArray[i] = 0; // init the array to zeroes
                while (readChar != ':' && sdCardFile.available()) { // loop through until we reach a colon. this indicates the end of "dataType"
                    asciiCodedHex[ind++] = readChar;
                    if (sdCardFile.available()) {
                        readChar = sdCardFile.read();
                    }
                }
                readInt = strtol(asciiCodedHex, NULL, 16); // convert the ascii coded hex into an int
                ind = 0;                                   // reset the index
                readChar = sdCardFile.read();
                // loop though until we get to a comment, a newline, or a semicolon. Any of these indicate the end of the data array
                while (readChar != '/' && readChar != '\n' && readChar != ';' && readChar != 0x0d && sdCardFile.available()) {
                    asciiCodedHex[0] = readChar;
                    if (sdCardFile.available()) {
                        asciiCodedHex[1] = sdCardFile.read();
                    }
                    asciiCodedHex[2] = '\0'; // have to set byte #3 as ascii NULL because we are only using 2 ascii characters at a time here
                    dataArray[ind++] = strtol(asciiCodedHex, NULL, 16);
                    if (sdCardFile.available()) {
                        readChar = sdCardFile.read();
                    }
                }
                // send the data to the scenesArr obj in PSRAM
                loadSceneDataIntoPSRAM(sceneNumber, readInt, dataArray, ind);
                if (readChar == '/') {
                    while (sdCardFile.available() && readChar != '\n')
                        readChar = sdCardFile.read(); // read through comment if present
                }
            }
            sdCardFile.close();
        } else {
            sdCardFile.close();
            err++;
        }
    }
    if (err) {
        debugserialPrint(1, "Load scenes error: ");
        debugserialPrintln(1, err);
        return false;
    }
    return true;
}

///\brief Parses data and stores it in the SceneObj::midiFT object. Should only be called as a part of loading scenes from the SD.
void loadSceneDataIntoPSRAM(unsigned int sceneNumber, unsigned int dataType, byte dataArray[], unsigned int arrayLength) {
    switch (dataType) {
    case 0x0000 ... 0x000f: { // main buttons top row text
        unsigned int buttonNumber = dataType & 0x000f;
        // dbgserPrint("setting button num ");
        // dbgserPrint(buttonNumber);
        // dbgserPrint(" text: ");
        if (arrayLength > 50) {
            arrayLength = 50;
        }
        for (uint8_t i = 0; i < arrayLength; i++) {
            midiFT.scenesArr[sceneNumber].mainButtons[buttonNumber].topRowText[i] = dataArray[i];
            // dbgserPrint(String(char(dataArray[i])));
        }
        // dbgserPrintln("-");

        // ensure that the final character in the char array is null.
        // if (midiFT.scenesArr[sceneNumber].mainButtons[buttonNumber].topRowText[arrayLength - 1] != '\0' && arrayLength < 50)
        // {
        //     midiFT.scenesArr[sceneNumber].mainButtons[buttonNumber].topRowText[arrayLength] = '\0';
        // }
        break;
    }
    case 0x0010 ... 0x001f: { // main buttons actions->action
        unsigned int buttonNumber = dataType & 0x000f;
        for (uint8_t i = 0; i < arrayLength; i++) {
            midiFT.scenesArr[sceneNumber].mainButtons[buttonNumber].Actions[i].action = buttonActions::ActionTypes(dataArray[i]);
            // dbgserPrint("setting action : ");
            // dbgserPrintln(buttonActions::ActionTypes(dataArray[i]));
        }
        break;
    }
    case 0x2000 ... 0x2fff: { // main buttons actions->actiondata[16]
        unsigned int buttonNum = dataType & 0x000f;
        unsigned int actionNumber = (dataType & 0x0ff0) >> 4;
        for (uint8_t i = 0; i < arrayLength; i++) {
            midiFT.scenesArr[sceneNumber].mainButtons[buttonNum].Actions[actionNumber].actionData[i] = dataArray[i];
        }
        break;
    }
    case 0x0030 ... 0x003f: { // output ports output mode
        unsigned int portNumber = dataType & 0x000f;
        midiFT.scenesArr[sceneNumber].output_ports[portNumber].out_mode = out_port_modes(dataArray[0]);
        break;
    }
    case 0x0040 ... 0x004f: { // output ports state
        unsigned int portNumber = dataType & 0x000f;
        midiFT.scenesArr[sceneNumber].output_ports[portNumber].state = out_port_state(dataArray[0]);
        break;
    }
    case 0x0050 ... 0x005f: { // external button / expression pedal input port mode
        unsigned int externalButtonNum = dataType & 0x000f;
        midiFT.scenesArr[sceneNumber].extButtons[externalButtonNum].Btn_Mode = ext_btn_modes(dataArray[0]);
        break;
    }
    case 0x0060 ... 0x006f: { // ext button / exp pedsal actions->action
        unsigned int externalButtonNum = dataType & 0x000f;
        for (uint8_t i = 0; i < arrayLength; i++) {
            midiFT.scenesArr[sceneNumber].extButtons[externalButtonNum].Actions[i].action = buttonActions::ActionTypes(dataArray[i]);
        }
        break;
    }
    case 0x4000 ... 0x4fff: { // ext buttons / exp pedal actions->actiondata[16]
        unsigned int externalButtonNum = dataType & 0x000f;
        unsigned int actionNumber = (dataType & 0x0ff0) >> 4;
        for (uint8_t i = 0; i < arrayLength; i++) {
            midiFT.scenesArr[sceneNumber].extButtons[externalButtonNum].Actions[actionNumber].actionData[i] = dataArray[i];
        }
        break;
    }
                          // case 0x0010 ... 0x01ff:
                          // {

                          //     break;
                          // }
    }
}

///\brief {Overloaded function} Save scene data to SD. If called with no sceneNumber param, the other version of this function
/// will be called instead and will save all scenes.
///\param sceneNumber ...um, this be self explanatory.
///\param save_outputPort_state If true, the state of the outputPort will be saved so that it can be set to the current state whenever
/// the scene is loaded. Defaults to false.
int saveSceneDataToSD(uint16_t sceneNumber, bool save_outputPort_state = false) {

    String fileName = "SCN";  // file name prefix
    String fileExt = ".TXT";  // file name suffix
    String fullFileName = ""; // a String to hold the concatenated filename
    char nameChars[13];
    int err = 0;
    fullFileName = fileName + sceneNumber + fileExt; // concatenate stuff to make the filename
    fullFileName.toCharArray(nameChars, 13);
    if (SD.exists(nameChars)) {
        SD.remove(nameChars);
    }
    sdCardFile = SD.open(nameChars, FILE_WRITE);
    if (sdCardFile) {
        for (uint8_t i = 0; i < 10; i++) { // main buttons
            // Button's top row text
            // sdCardFile.print(byte(0x0000 | i), HEX); // use either this line or the next. not both.
            sdCardFile.printf("%04X", uint16_t(i));
            sdCardFile.print(':');
            for (uint8_t c = 0; c < 50; c++) {
                sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].mainButtons[i].topRowText[c]);
            }
            // sdCardFile.print('\0');
            sdCardFile.print(';');

            // Button's Array of actions
            sdCardFile.printf("%04X", uint16_t(0x0010 | i));
            sdCardFile.print(':');
            for (uint8_t u = 0; u < 32; u++) {
                // sdCardFile.print(byte(midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u].action), HEX); // use either this line or the next. not both.
                sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u].action);
                if (midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u].action == buttonActions::NULL_Action)
                    break;
            }
            sdCardFile.print(';');

            // Button Action's array of data
            for (uint8_t u = 0; u < 32; u++) {
                sdCardFile.printf("%04X", (uint16_t(i) | uint16_t(u << 4)) | 0x2000);
                sdCardFile.print(":");
                for (uint8_t j = 0; j < 16; j++) {
                    // sdCardFile.print(midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u].actionData[j], HEX); // use either this line or the next. not both.
                    sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u].actionData[j]);
                }
                sdCardFile.print(";");
                if (u < 31) {
                    if (midiFT.scenesArr[sceneNumber].mainButtons[i].Actions[u + 1].action == buttonActions::NULL_Action)
                        break;
                }
            }
            // Newline
            sdCardFile.println("");
        }

        for (uint8_t i = 0; i < 4; i++) { // output ports
            sdCardFile.printf("%04X", uint16_t(i | 0x0030));
            sdCardFile.print(":");
            sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].output_ports[i].out_mode);
            sdCardFile.print(";");
            if (save_outputPort_state) {
                sdCardFile.printf("%04X", uint16_t(i | 0x0040));
                sdCardFile.print(":");
                sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].output_ports[i].state);
                sdCardFile.print(";");
            }
            sdCardFile.println("");
        }

        for (uint8_t i = 0; i < 8; i++) { // external buttons
            sdCardFile.printf("%04X", uint16_t(i | 0x0050));
            sdCardFile.print(":");
            sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].extButtons[i].Btn_Mode);
            sdCardFile.print(";");

            // Ext Button's Array of actions
            sdCardFile.printf("%04X", uint16_t(0x0060 | i));
            sdCardFile.print(':');
            for (uint8_t u = 0; u < 32; u++) {
                // sdCardFile.print(byte(midiFT.scenesArr[sceneNumber].extButtons[i].Actions[u].action), HEX); // use either this line or the next. not both.
                sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].extButtons[i].Actions[u].action);
                if (midiFT.scenesArr[sceneNumber].extButtons[i].Actions[u].action == buttonActions::NULL_Action)
                    break;
            }
            sdCardFile.print(';');

            // Ext Button Action's array of data
            for (uint8_t u = 0; u < 32; u++) {
                sdCardFile.printf("%04X", (uint16_t(i) | uint16_t(u << 4)) | 0x4000);
                sdCardFile.print(":");
                for (uint8_t j = 0; j < 16; j++) {
                    sdCardFile.printf("%02X", midiFT.scenesArr[sceneNumber].extButtons[i].Actions[u].actionData[j]);
                }
                sdCardFile.print(";");
                if (u < 31) {
                    if (midiFT.scenesArr[sceneNumber].extButtons[i].Actions[u + 1].action == buttonActions::NULL_Action)
                        break;
                }
            }
            sdCardFile.println("");
        }
    } else {
        err++;
    }
    sdCardFile.close();
    return err;
}

///\brief {Overloaded function} Save scene data to SD. Will save all scenes to the SD.
///\param save_outputPort_state If true, the state of the outputPort will be saved so that it can be set to the current state whenever
/// the scene is loaded. Defaults to false.
int saveSceneDataToSD(bool save_outputPort_state = false) {
    debugserialPrintln(5, "Starting file write loop...");
    int err = 0;
    for (uint16_t i = 0; i < NUMBER_OF_SCENES; i++)
        saveSceneDataToSD(i, save_outputPort_state) == 0 ? err += 0 : err++;
    debugserialPrintln(5, "File write loop copmpleted. Returning");
    return err;
}

///\brief Load general preferences from prefs file.
///\return True if successful.
bool loadPrefsFromFile() {
    sdCardFile = SD.open("prefs.txt");
    if (sdCardFile) {
        uint8_t line = 0;
        while (sdCardFile.available()) {
            char readChar = sdCardFile.read();
            if (readChar == 0x0d) {
                // do nothing, skip carriage return
            } else if (readChar == '\n' || readChar == '/') { // new line or reached comment
                line++;
            } else {
                switch (line) {
                case 0: { // MIDI to USB passthrough setting
                    for (int i = 0; i < 4; i++) {
                        while (readChar != ',' && readChar != '/') {
                            if (readChar == 't') {
                                preferences.passThrough.MIDItoUSB[i] = true;
                            } else if (readChar == 'f') {
                                preferences.passThrough.MIDItoUSB[i] = false;
                            } else {
                                sdCardFile.close();
                                return false;
                            }
                            readChar = sdCardFile.read();
                        }
                        readChar = sdCardFile.read();
                    }
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 1: { // USB to MIDI passthrough setting
                    for (int i = 0; i < 4; i++) {
                        while (readChar != ',' && readChar != '/') {
                            if (readChar == 't') {
                                preferences.passThrough.USBtoMIDI[i] = true;
                            } else if (readChar == 'f') {
                                preferences.passThrough.USBtoMIDI[i] = false;
                            } else {
                                sdCardFile.close();
                                return false;
                            }
                            readChar = sdCardFile.read();
                        }
                        readChar = sdCardFile.read();
                    }
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 2: { // MIDI to MIDI passthrough setting
                    for (int i = 0; i < 4; i++) {
                        while (readChar != ',' && readChar != '/') {
                            char temp[2] = { readChar, '\0' };
                            preferences.passThrough.MIDItoMIDI[i] = atoi(temp);
                            readChar = sdCardFile.read();
                        }
                        readChar = sdCardFile.read();
                    }
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 3: { // LCD backlight brightness
                    char intBuf[4];
                    int i = 0;
                    while (readChar != '/' && readChar != '\n') {
                        intBuf[i++] = readChar;
                        readChar = sdCardFile.read();
                    }
                    intBuf[i] = '\0';
                    int num = atoi(intBuf);
                    preferences.backlightBrightness = byte(num);
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 4: { // Hardware midi port channel setting
                    // Used for thru filtering when thru settings is midi::Thru::SameChannel or midi::Thru::DifferentChannel
                    // Sets channel for selected port. 
                    for (int i = 0; i < 4; i++) {
                        while (readChar != ',' && readChar != '/') {
                            char temp[2] = { readChar, '\0' };
                            preferences.channel[i] = atoi(temp);
                            readChar = sdCardFile.read();
                        }
                        readChar = sdCardFile.read();
                    }
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 5: { // arrow positions
                    char intBuf[2];
                    int i = 0;
                    while (readChar != '/' && readChar != '\n') {
                        intBuf[i++] = readChar;
                        readChar = sdCardFile.read();
                    }
                    intBuf[i] = '\0';
                    int num = atoi(intBuf);
                    preferences.arrowPos = PrefsObj::ArrowPositions(num);
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 6: { // Number of scenes 
                    char intBuf[6];
                    int i = 0;
                    while (readChar != '/' && readChar != '\n') {
                        intBuf[i++] = readChar;
                        readChar = sdCardFile.read();
                    }
                    intBuf[i] = '\0';
                    int num = atoi(intBuf);
                    preferences.totalNumberOfScenes = num <= 1500 ? num : 1500;
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 7: { // LCD update time
                    char intBuf[6];
                    int i = 0;
                    while (readChar != '/' && readChar != '\n') {
                        intBuf[i++] = readChar;
                        readChar = sdCardFile.read();
                    }
                    intBuf[i] = '\0';
                    int num = atoi(intBuf);
                    preferences.LCD_TextUpdateTime = num;
                    while (readChar != '\n' && sdCardFile.available())
                        readChar = sdCardFile.read(); // read through comment if present
                    break;
                }
                case 8: {
                    // maybe settings to enable debuging???
                    break;
                }
                }
                line++;
            }
        }
        sdCardFile.close();
        picoOBJ.setConfig();
        return true;
    }
    sdCardFile.close();
    return false;
}

///\brief Set midi thru setting in the MIDI library objects according the general preferences currently in RAM.
void updateHardwareMIDIthru() {
    for (uint8_t i = 0; i < 4; i++) {
        switch (preferences.passThrough.MIDItoMIDI[i]) {
        case midi::Thru::Full:
            HW_midi[i].turnThruOn();
            break;
        case midi::Thru::Off:
            HW_midi[i].turnThruOff();
            break;
        case midi::Thru::SameChannel:
            HW_midi[i].turnThruOn();
            HW_midi[i].setThruFilterMode(midi::Thru::Mode::SameChannel);
            break;
        case midi::Thru::DifferentChannel:
            HW_midi[i].turnThruOn();
            HW_midi[i].setThruFilterMode(midi::Thru::Mode::DifferentChannel);
            break;
        }
    }
}

///\brief Scrolls the text on the "top rows" of each LCD one position. Should be called at a fixed interval to ensure smooth scrolling.
void LCDS_updateText() {
    char tempCharArr[19];

    char tempCharArrWithSpaces[53];

    for (uint8_t buttonID = 0; buttonID < 10; buttonID++) {
        for (uint8_t c = 0; c < 18; c++)
            tempCharArr[c] = ' ';
        // make a temp version of the text that has three spaces after the end of the text.
        uint8_t iterator = 0;
        for (; iterator < 50; iterator++) {
            if (currentSceneInRAM.mainButtons[buttonID].topRowText[iterator] == '\0')
                break;
            tempCharArrWithSpaces[iterator] = currentSceneInRAM.mainButtons[buttonID].topRowText[iterator];
        }
        for (uint8_t iterator_plus3 = iterator + 3; iterator < iterator_plus3; iterator++)
            tempCharArrWithSpaces[iterator] = ' ';
        uint8_t stringLength = iterator;
        if (stringLength > 21)
            for (uint8_t i = 0; i < 18; i++)
                if (topRowTextPositions[buttonID] + i >= stringLength)
                    tempCharArr[i] = tempCharArrWithSpaces[i + topRowTextPositions[buttonID] - stringLength];
                else
                    tempCharArr[i] = tempCharArrWithSpaces[i + topRowTextPositions[buttonID]];
        else
            for (uint8_t i = 0; i < stringLength - 3; i++)
                tempCharArr[i] = tempCharArrWithSpaces[i];

        /*
        LCD id:
            Each LCD displays the text for two buttons. Buttons 1 and 2 are on LCD #1 (id 0), 3 and 4 are on LCD #2 (id 1), etc.
            This pairing means we can drop the LSB of the button number and get the appropriate LCD id.

        Char Array:
            18 character array that holds the text to be sent to the LCD

        Num of Chars:
            Always 18 since tempCharArr is 18 chars long. (plus a null term but thats not needed, maybe.)

        Column:
            The starting column is calculated based off the arrowPos(ition) preference.
            First determine if buttonID is odd or even. (buttonID&1)==1?....
            If odd, shift arrowPos(ition) variable right 1 bit.
            If even, shift none.
                preferences.arrowPos>>({buttonID is odd}?1:0)
            This leave us with a value that indicates whether the button's relative arrow (displayed on the screen) is on the left or right.
            The LSB is 1 if the arrow is on the left, and 0 if on the right.
                (Arrow position left or right)&1
            If the arrow is on the right, then column is 0. If on the left, column is 2.
                (Left (i.e. 0) or Right (i.e. 1)) * 2

        Row:
            Row is either 2 or 0 based on whether the buttonID is odd or even, respectively
                (buttonID&1)==1?2:0)
                (buttonID is odd?2:0)
        */
        bool isBtnIdOdd = (buttonID & 1) == 1;
        LCDs_printText(buttonID >> 1, tempCharArr, 18, ((preferences.arrowPos >> (isBtnIdOdd ? 1 : 0)) & 1) * 2, isBtnIdOdd ? 2 : 0, true);
        topRowTextPositions[buttonID] < stringLength ? topRowTextPositions[buttonID]++ : topRowTextPositions[buttonID] = 0;
    }
}

///\brief Reset "top row text" scroll position to 0 and load {sceneNumber} into RAM from PSRAM
///\param sceneNumber It's the number of the scene you want to load. 0 indexed.
void setCurrentScene(int sceneNumber) {
    if (sceneNumber > (NUMBER_OF_SCENES - 1)) {
        return;
    }
    currentScene = sceneNumber;
    // resetLCD's text position
    for (uint8_t i = 0; i < 10; i++) {
        topRowTextPositions[i] = 0;
    }
    for (int u = 0; u < 8; u++) {
        for (int p = 0; p < 32; p++) {
            currentSceneInRAM.extButtons[u].Actions[p].action = midiFT.scenesArr[sceneNumber].extButtons[u].Actions[p].action;
            for (int q = 0; q < 16; q++) {
                currentSceneInRAM.extButtons[u].Actions[p].actionData[q] = midiFT.scenesArr[sceneNumber].extButtons[u].Actions[p].actionData[q];
            }
        }
        currentSceneInRAM.extButtons[u].Btn_Mode = midiFT.scenesArr[sceneNumber].extButtons[u].Btn_Mode;
    }
    for (int u = 0; u < 4; u++) {
        currentSceneInRAM.output_ports[u].out_mode = midiFT.scenesArr[sceneNumber].output_ports[u].out_mode;
        currentSceneInRAM.output_ports[u].state = midiFT.scenesArr[sceneNumber].output_ports[u].state;
    }
    for (int u = 0; u < 10; u++) {
        for (int p = 0; p < 32; p++) {
            currentSceneInRAM.mainButtons[u].Actions[p].action = midiFT.scenesArr[sceneNumber].mainButtons[u].Actions[p].action;
            for (int q = 0; q < 16; q++) {
                currentSceneInRAM.mainButtons[u].Actions[p].actionData[q] = midiFT.scenesArr[sceneNumber].mainButtons[u].Actions[p].actionData[q];
            }
        }
        for (int p = 0; p < 50; p++) {
            currentSceneInRAM.mainButtons[u].topRowText[p] = midiFT.scenesArr[sceneNumber].mainButtons[u].topRowText[p];
        }
    }
}

///\brief  Forwards serial data between the USB serial port and the ESP8266 serial port. Allows programming the ESP8266 in-circuit. Closed
/// "button 9" circuit to exit and return to loop(). Closed "button 10" circuit to reset ESP and put it into programming mode. To get the
/// ESP out of programming mode, power cycle everything.
void enter_ESP8266_ProgrammingMode(unsigned long baud) {
    // delay(2000);
    digitalWrite(ESP_BOOT_MODE, HIGH);
    digitalWrite(ESP_RESET, LOW);
    delay(250);
    digitalWrite(ESP_RESET, HIGH);
    delay(500);
    // USB_Serial.begin(115200);
    ESP8266_Serial.end();
    delay(250);
    ESP8266_Serial.begin(baud);
    USB_Serial.println("Ready.");
    while (1) {
        digitalWrite(ESP_RESET, !USB_Serial.rts());
        digitalWrite(ESP_BOOT_MODE, !USB_Serial.dtr());
        if (digitalRead(22) == LOW) {
            // reset ESP and enter programming mode
            digitalWrite(ESP_RESET, LOW);
            digitalWrite(ESP_BOOT_MODE, LOW);
            delay(250);
            digitalWrite(ESP_RESET, HIGH);
            delay(250);
            while (digitalRead(22) == LOW) {
            } // wait for pin to go high again
            digitalWrite(ESP_BOOT_MODE, HIGH);
        }
        if (digitalRead(23) == LOW) {
            // exit ESP mode
            digitalWrite(ESP_RESET, LOW);
            delay(250);
            digitalWrite(ESP_RESET, HIGH);
            goto enter_ESP8266_ProgrammingMode_return;
        }
        if (USB_Serial.available()) {
            ESP8266_Serial.write(USB_Serial.read());
        }
        if (ESP8266_Serial.available()) {
            USB_Serial.write(ESP8266_Serial.read());
        }
    }
enter_ESP8266_ProgrammingMode_return:
    while (USB_Serial.available())
        USB_Serial.read();
    while (ESP8266_Serial.available())
        ESP8266_Serial.read();
    return;
}

void getESPSerialData(uint8_t retVal[]) {
    // dbgserPrintln("Now in handleESP8266Serial().");
    uint8_t err = 0;
    uint8_t first6[6];
    uint8_t theRest[18];
    uint8_t packetSize = 0;
    // dbgserPrintln("Now in while loop.");
    // uint8_t inByte = ESP8266_Serial.read();
    ESP8266_Serial.setTimeout(2000);
    if (ESP8266_Serial.readBytes(first6, 6) == 0) {
        err = SERIAL_ERROR::TIMEOUT;
    }
    // break;
    // dbgserPrint("first6[0]: ");
    // dbgserPrintln_T(first6[0],HEX);
    // dbgserPrint("first6[0] &0xf0: ");
    // dbgserPrintln_T((first6[0] & 0xf0),HEX);
    // dbgserPrint("first6[0]: ");
    // dbgserPrintln_T(first6[0],HEX);
    if (((first6[0] & 0xf0) == 0xa0) && (err == SERIAL_ERROR::NONE)) {
        // short message
        packetSize = 6;

        // copy data into ESP_ShortMessage object
        for (uint8_t i = 0; i < 5; i++)
            ESP_ShortMessage[i] = first6[i];

        // save the crc that was sent
        uint8_t sentCRC = first6[5];
        uint8_t calcCRC = ESP_ShortMessage.CalculateCRC();

        // compare the crc that was sent with a calculated crc
        if (sentCRC != calcCRC)
            err = SERIAL_ERROR::CRC8_MISMATCH;
    } else if (err == SERIAL_ERROR::NONE) {
        // full packet
        packetSize = 24;

        // get the rest of the packet..
        if (ESP8266_Serial.readBytes(theRest, 18) != 18)
            err = SERIAL_ERROR::TIMEOUT;
        // break;
        // copy the packet into the ESP_SerPacket object
        uint8_t i = 0;
        for (; i < 6; i++)
            ESP_SerPacket[i] = first6[i];
        for (; i < 24; i++)
            ESP_SerPacket[i] = theRest[i - 6];

        // save the sent crc32
        uint32_t sentCRC = ESP_SerPacket.crc_32;

        // compare the sent CRC32 with a calculated CRC32
        if (sentCRC != ESP_SerPacket.CalculateCRC())
            err = SERIAL_ERROR::CRC32_MISMATCH;
    }
    // dbgserPrint("ESP sent ");
    // dbgserPrintln(packetSize == 6 ? "a short message." : (packetSize == 24 ? "a full size packet." : "poorly formatted data."));
    // dbgserPrint("CRC compare result: ");
    // dbgserPrintln(err == SERIAL_ERROR::CRC32_MISMATCH ? "CRC32 mismatch." : (err == SERIAL_ERROR::CRC8_MISMATCH ? "CRC8 mismatch." : "equal."));

    if (err & SERIAL_ERROR::TIMEOUT)
        dbgserPrintln("timout...");
    // if(err)
    //     dbgserPrintln(err);
    retVal[0] = packetSize;
    retVal[1] = err;
}

///\brief TODO
void handleESP8266Serial() {
    uint8_t pSizeAndErr[] = { 255,255 };
    getESPSerialData(pSizeAndErr);

    if (pSizeAndErr[0] == 6 && pSizeAndErr[1] == 0) {
        uint8_t command = ESP_ShortMessage.StartSequence_8 & 0x0f;
        // dbgserPrintln(command);
        switch (command) {
        case ESP_SERIAL_COMMANDS_Message::RequestForHashCompare: {
            // sendDataToESP(ESP_SERIAL_COMMANDS_Message::OkToStartSendingData,)
            break;
        }
        case ESP_SERIAL_COMMANDS_Message::RequestForSceneFile: {
            dbgserPrintln("request for scene file");
            // dbgserPrintln("request for scene file.");
            unsigned int err = 0;
            int requestedSceneNum = int(ESP_ShortMessage.data[0] | (ESP_ShortMessage.data[1] << 8));
            String fileName = "SCN";  // file name prefix
            String fileExt = ".TXT";  // file name suffix
            String fullFileName = ""; // a String to hold the concatenated filename
            char nameChars[15];
            fullFileName = fileName + requestedSceneNum + fileExt; // concatenate stuff to make the filename
            fullFileName.toCharArray(nameChars, 15);
            sdCardFile = SD.open(nameChars);
            unsigned int sequenceNumber = 0;
            unsigned long byteNumber = 0;
            uint8_t outArr[16];

            // dbgserPrintln(fullFileName);
            if (sdCardFile) {
                // dbgserPrintln("sdCardFile exists.");
                unsigned long size = sdCardFile.size();
                outArr[0] = uint8_t(size);
                outArr[1] = uint8_t(size >> 8);
                outArr[2] = uint8_t(size >> 16);
                outArr[3] = uint8_t(size >> 24);
                sendDataToESP((ESP_SERIAL_DataType::FileSize | ESP_SERIAL_COMMANDS_Message::isMessageNotPacket), outArr, 4, false);
                uint8_t count = 0;
                while ((ESP8266_Serial.available() == 0) && (count < 255)) {
                    count++;
                    delay(1);
                }
                getESPSerialData(pSizeAndErr);
                uint8_t newCommand = ESP_ShortMessage.StartSequence_8 & 0x0f;
                if (newCommand != ESP_SERIAL_COMMANDS_Message::OkToStartSendingData || pSizeAndErr[1] > 0) {
                    err += 8;
                    break;
                }
                bool looping = true;
                while (looping) {
                    byte readByte;
                    bool isLast = false;
                    for (uint8_t i = 0;i < 16;i++) {
                        if (sdCardFile.available()) {
                            readByte = sdCardFile.read();
                            outArr[i] = readByte;
                            byteNumber++;
                        } else {
                            outArr[i] = 0;
                            isLast = true;
                        }
                    }
                    if (!isLast) {
                        // command to send is 0xSSCD where SS is sequence number, C is start or continue, and D is scene data
                        // dbgserPrint("sending some data. bytenumber: ");
                        // dbgserPrintln(byteNumber);
                        sendDataToESP(((byteNumber == 16 ? ESP_SERIAL_COMMANDS_Packet::StartSendData : ESP_SERIAL_COMMANDS_Packet::ContinueSendData) | ESP_SERIAL_COMMANDS_Packet::SaveSceneData) | (sequenceNumber << 8), outArr, 16, true);
                        count = 0;
                        while ((ESP8266_Serial.available() == 0) && (count < 255)) {
                            count++;
                            delay(1);
                        }
                        getESPSerialData(pSizeAndErr);
                        if (pSizeAndErr[1] > 0) {
                            err += 16;
                            break;
                        }
                        newCommand = ESP_ShortMessage.StartSequence_8 & 0x0f;
                        if (newCommand != ESP_SERIAL_COMMANDS_Message::OkToContinueSendingData) {
                            err += 8;
                            break;
                        }
                    } else {
                        // dbgserPrintln("sending the last data.");
                        sendDataToESP((ESP_SERIAL_COMMANDS_Packet::EndSendData | ESP_SERIAL_COMMANDS_Packet::SaveSceneData), outArr, 16, true);
                        looping = false;
                    }
                    sequenceNumber++;
                }
                sdCardFile.close();
            } else err++;
            if (err != 0) {
                dbgserPrint("err: ");
                dbgserPrintln(err);
            }
            break;
        }
        case ESP_SERIAL_COMMANDS_Message::RequestForTotalNumberOfScene: {
            dbgserPrintln("request for number of scenes");
            uint8_t outArr[4];
            unsigned long totalNumberOfScene = NUMBER_OF_SCENES;
            outArr[0] = uint8_t(totalNumberOfScene);
            outArr[1] = uint8_t(totalNumberOfScene >> 8);
            outArr[2] = uint8_t(totalNumberOfScene >> 16);
            outArr[3] = uint8_t(totalNumberOfScene >> 24);
            sendDataToESP((ESP_SERIAL_DataType::TotalNumOfScenes | ESP_SERIAL_COMMANDS_Message::isMessageNotPacket), outArr, 4, false);
            break;
        }
        case ESP_SERIAL_COMMANDS_Message::RequestToSaveSceneFile: {
            dbgserPrintln("request to save scene data");
            unsigned int err = 0;
            uint8_t newCommand = 0;
            uint8_t outArr[4];
            outArr[0] = 0;
            outArr[1] = 0;
            outArr[2] = 0;
            outArr[3] = 0;
            int sceneNumToSave = int(ESP_ShortMessage.data[0] | (ESP_ShortMessage.data[1] << 8));
            String fileName = "SCN";  // file name prefix
            String fileExt = ".TXT";  // file name suffix
            String fullFileName = ""; // a String to hold the concatenated filename
            char nameChars[15];
            fullFileName = fileName + sceneNumToSave + fileExt; // concatenate stuff to make the filename
            fullFileName.toCharArray(nameChars, 15);
            if (SD.exists(nameChars)) {
                dbgserPrintln("file exists, deleting.");
                SD.remove(nameChars);
            }
            sdCardFile = SD.open(nameChars, FILE_WRITE);
            if (sdCardFile) {
                dbgserPrintln("sdCardFile opened for writing. Continuing...");
                sendDataToESP(ESP_SERIAL_COMMANDS_Message::OkToStartSendingData | ESP_SERIAL_COMMANDS_Message::isMessageNotPacket, outArr, false);
                getESPSerialData(pSizeAndErr);
                if ((pSizeAndErr[0] != 24) || (pSizeAndErr[1] > 0)) {
                    err += pSizeAndErr[1];
                    dbgserPrint("There was a problem with rec's data. err: ");
                    dbgserPrintln_T(err, HEX);
                } else {
                    newCommand = ESP_SerPacket.startSequence_32 & 0xff;
                    if (newCommand == (ESP_SERIAL_COMMANDS_Packet::StartSendData | ESP_SERIAL_COMMANDS_Packet::SaveSceneData)) {
                        dbgserPrintln("Rec'd data seems ok. Saving to file.");
                        do {
                            dbgserPrint(".");
                            for (uint8_t i = 0;i < 16;i++) {
                                sdCardFile.write(ESP_SerPacket.data[i]);
                            }
                            sendDataToESP(ESP_SERIAL_COMMANDS_Message::OkToContinueSendingData | ESP_SERIAL_COMMANDS_Message::isMessageNotPacket, outArr, 0, false);
                            getESPSerialData(pSizeAndErr);
                            if ((pSizeAndErr[0] != 24) || (pSizeAndErr[1] > 0)) {
                                newCommand = 0;
                                dbgserPrintln("There was a problem with Rec'd data. line 1203");
                            } else {
                                newCommand = ESP_SerPacket.startSequence_32 & 0xff;
                            }
                        } while (newCommand == (ESP_SERIAL_COMMANDS_Packet::ContinueSendData | ESP_SERIAL_COMMANDS_Packet::SaveSceneData));
                        dbgserPrintln("Most recent rec's command was not a continue command. Checking for end command.");
                        if (newCommand == (ESP_SERIAL_COMMANDS_Packet::EndSendData | ESP_SERIAL_COMMANDS_Packet::SaveSceneData)) {
                            dbgserPrintln("End command rec'd. saving data to file and closing.");
                            for (uint8_t i = 0;i < 16;i++) {
                                dbgserPrint(".");
                                sdCardFile.write(ESP_SerPacket.data[i]);
                                if (ESP_SerPacket.data[i + 1] == 0) {
                                    i = 16;
                                }
                            }
                            err = 0;
                        } else {
                            err = 128;
                            dbgserPrintln("Did not rec expected end command. line 1221");
                        }
                    } else {
                        err = 255;
                        dbgserPrintln("did not rec expected start command.");
                    }
                }
            } else {
                dbgserPrintln("Unable to open sdCardFile. Send error command.");
                sendDataToESP(ESP_SERIAL_COMMANDS_Packet::EndSendData | SERIAL_ERROR::SD_FILE_ERROR, outArr, 0, true);
            }
            sdCardFile.close();
            loadScenesFromFile(sceneNumToSave);
            setCurrentScene(currentScene);
            break;
        }
        case ESP_SERIAL_COMMANDS_Message::RequestForPreferences: {
            // @todo 
            // Can probably copy most of the code from "request for scene file"
            dbgserPrintln("request for prefs file");
            unsigned int err = 0;
            char nameChars[] = "prefs.txt";
            sdCardFile = SD.open(nameChars);
            unsigned int sequenceNumber = 0;
            unsigned long byteNumber = 0;
            uint8_t outArr[16];
            if (sdCardFile) {
                unsigned long size = sdCardFile.size();
                outArr[0] = uint8_t(size);
                outArr[1] = uint8_t(size >> 8);
                outArr[2] = uint8_t(size >> 16);
                outArr[3] = uint8_t(size >> 24);
                // dbgserPrintln("sending size to ESP");
                sendDataToESP((ESP_SERIAL_DataType::FileSize | ESP_SERIAL_COMMANDS_Message::isMessageNotPacket), outArr, 4, false);
                uint8_t count = 0;
                while ((ESP8266_Serial.available() == 0) && (count < 255)) {
                    count++;
                    delay(1);
                }
                // dbgserPrintln("get command form esp");
                getESPSerialData(pSizeAndErr);
                // dbgserPrint("command \"");
                uint8_t newCommand = ESP_ShortMessage.StartSequence_8 & 0x0f;
                // dbgserPrint_T(newCommand, HEX);
                // dbgserPrintln("\" received.");
                if (newCommand != ESP_SERIAL_COMMANDS_Message::OkToStartSendingData || pSizeAndErr[1] > 0) {
                    err += 8;
                    break;
                }
                bool looping = true;
                while (looping) {
                    byte readByte;
                    bool isLast = false;
                    for (uint8_t i = 0;i < 16;i++) {
                        if (sdCardFile.available()) {
                            readByte = sdCardFile.read();
                            outArr[i] = readByte;
                            byteNumber++;
                        } else {
                            outArr[i] = 0;
                            isLast = true;
                        }
                    }
                    if (!isLast) {
                        // command to send is 0xSSCD where SS is sequence number, C is start or continue, and D is scene data
                        // dbgserPrint("sending some data. bytenumber: ");
                        // dbgserPrintln(byteNumber);
                        sendDataToESP(((byteNumber == 16 ? ESP_SERIAL_COMMANDS_Packet::StartSendData : ESP_SERIAL_COMMANDS_Packet::ContinueSendData) | ESP_SERIAL_COMMANDS_Packet::SavePreferences) | (sequenceNumber << 8), outArr, 16, true);
                        count = 0;
                        while ((ESP8266_Serial.available() == 0) && (count < 255)) {
                            count++;
                            delay(1);
                        }
                        getESPSerialData(pSizeAndErr);
                        if (pSizeAndErr[1] > 0) {
                            err += 16;
                            break;
                        }
                        newCommand = ESP_ShortMessage.StartSequence_8 & 0x0f;
                        if (newCommand != ESP_SERIAL_COMMANDS_Message::OkToContinueSendingData) {
                            err += 8;
                            break;
                        }
                    } else {
                        // dbgserPrintln("sending the last data.");
                        sendDataToESP((ESP_SERIAL_COMMANDS_Packet::EndSendData | ESP_SERIAL_COMMANDS_Packet::SavePreferences), outArr, 16, true);
                        looping = false;
                    }
                    sequenceNumber++;
                }
                sdCardFile.close();
            } else err++;
            if (err != 0) {
                dbgserPrint("err: ");
                dbgserPrintln(err);
            }
            break;
        }
        case ESP_SERIAL_COMMANDS_Message::RequestToSavePrefsFile: {
            //@todo 
            loadPrefsFromFile(); //@note This should be the last thing to happen after saving preferences. 
            break;
        }
        default:
            dbgserPrintln("No Matching command found.");
            break;
        }
    }
    return;
}

///\brief Sends a command + data to ESP8266.
///\return True if serial write buffer had enough space to not block the operation. false if writing to the serial port
/// would have blocked. If false, no data was written to serial port.
///\param command The command to send.
///\param data Or dataArr: single byte or array of bytes
///\param len length of the data array. If len is too large to fit in packet or message, return false.
///\param isFullPacket bool: true to send full 24 byte packet, false to send 6 byte message
bool sendDataToESP(uint16_t command, uint8_t dataArr[], uint8_t len, bool isFullPacket) {
    // dbgserPrintln("sending data to ESP.");
    // dbgserPrint("command (HEX): ");
    // dbgserPrintln_T(command,HEX);
    // dbgserPrint("dataArr: ");
    for (int i = 0;i < len;i++) {
        // dbgserPrint_T(dataArr[i],HEX);
        // dbgserPrint(" : ");
    }
    // dbgserPrintln("");
    // dbgserPrint("len (DEC): ");
    // dbgserPrintln_T(len,DEC);
    // dbgserPrint("isFullPacket (bool): ");
    // dbgserPrintln(isFullPacket);
    if (len > (isFullPacket ? 16 : 4)) {
        // dbgserPrintln("len is too big. returning...");
        return false;
    }
    uint8_t fullSizeDataArray[16];
    uint8_t i = 0;
    for (; i < len; i++)
        fullSizeDataArray[i] = dataArr[i];
    for (; i < 16; i++)
        fullSizeDataArray[i] = 0;
    if (isFullPacket) {
        if (ESP8266_Serial.availableForWrite() < 24) {
            // dbgserPrintln("not enough space in buffer to not block;");
            // return false; // return false if hardware buffer cant hold the entire packet.
        }
        ESP_SerPacket.startSequence_32 = 0x0000aa55 | (command << 16);
        for (uint8_t i = 0; i < 16; i++)
            ESP_SerPacket.data[i] = fullSizeDataArray[i];
        ESP_SerPacket.CalculateCRC();
        for (uint8_t i = 0; i < 24; i++)
            ESP8266_Serial.write(ESP_SerPacket[i]);
    } else {
        if (ESP8266_Serial.availableForWrite() < 6) {
            // dbgserPrintln("not enough space in buffer to not block;");
            // return false; // return false if hardware buffer cant hold the entire packet.
        }
        ESP_ShortMessage.StartSequence_8 = uint8_t(command & 0x00ff);
        for (uint8_t i = 0; i < 4; i++)
            ESP_ShortMessage.data[i] = fullSizeDataArray[i];
        ESP_ShortMessage.CalculateCRC();
        for (uint8_t i = 0; i < 6; i++)
            ESP8266_Serial.write(ESP_ShortMessage[i]);
    }
    return true;
}

bool sendDataToESP(uint16_t command, uint8_t data, bool isFullPacket) {
    uint8_t newArray[isFullPacket ? 16 : 4];
    newArray[0] = data;
    for (uint8_t i = 1; i < isFullPacket ? 16 : 4; i++)
        newArray[i] = 0;
    return sendDataToESP(command, newArray, isFullPacket ? 16U : 4U, isFullPacket);
}

bool sendDataToESP(uint16_t command, uint8_t dataArr[], bool isFullPacket) {
    return sendDataToESP(command, dataArr, isFullPacket ? 16U : 4U, isFullPacket);
}

void setup() {
    for (uint8_t i = 0;i < 10;i++) {
        mainActionQ[i].setPicoObj(picoOBJ);
    }
    for (uint8_t i = 0;i < 8;i++) {
        extActionQ[i].setPicoObj(picoOBJ);
    }
    for (uint8_t i = 0; i < 10; i++) {
        BtnDebouncer[i].attach(BtnPins[i], INPUT_PULLUP);
        BtnDebouncer[i].interval(5);
    }

    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);
    digitalWrite(31, HIGH);
    digitalWrite(32, HIGH);
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);

    debugSerialBegin(115200);
    ESP8266_Serial.begin(1500000); // Connection to ESP8266

    midiFT.initValues();                   // initialize all the values in the scene data in PSRAM.
    midiFT.scenesArr[0].resetToDefaults(); // Set all values in Scene 1 to defaults
    currentSceneInRAM.resetToDefaults();
#ifdef ESP_PROGRAMMING_MODE
    enter_ESP8266_ProgrammingMode(115200);
#endif
    if (EEPROM.read(0) == 0x0f && EEPROM.read(1) == 0x8b && EEPROM.read(2) == 0xd2 && EEPROM.read(3) == 0x98) {
        EEPROM.write(0, 0);
        EEPROM.write(1, 0);
        EEPROM.write(2, 0);
        EEPROM.write(3, 0);
        enter_ESP8266_ProgrammingMode(115200);
    }

    if (!SD.begin(BUILTIN_SDCARD)) {
        sdCardInit = false;
        debugserialPrintln(1, "Error initializing SD card.");
    } else {
        sdCardInit = true;
        debugserialPrintln(4, "SD card initialized.");
    }
    debugserialPrintln(4, "Loading prefs from file");
    if (!loadPrefsFromFile())
        debugserialPrintln(1, "Error loading prefs from file");

    // ensure that LCD's text update time is reasonable
    if ((preferences.LCD_TextUpdateTime < 100) || (preferences.LCD_TextUpdateTime > 1000))
        preferences.LCD_TextUpdateTime = 300;
    Wire.begin(); // join i2c bus (address optional for master)
    Wire.setClock(400000);
    setupLCDs();
    char someChars[] = "midiFT booting";
    LCDs_printText(0, someChars, 14, 2, 0);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////  Save all scenes to SD card. Here for testing.
    // saveSceneDataToSD();///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    debugserialPrintln(4, "Loading scenes from files");

    if (!loadScenesFromFile())
        debugserialPrintln(1, "Error loading scenes from file");

    for (uint8_t i = 0; i < 4; i++)
        HW_midi[i].begin(MIDI_CHANNEL_OMNI);
    updateHardwareMIDIthru();                          // sets thru enable based on preferences.passThrough.MIDItoMIDI[]
    delay(500);                                        // give other controllers a chance to boot
    setLCDbrightness(preferences.backlightBrightness); // setLCDbrightness(getLCDbrightnessPref());

    // load scene #1
    setCurrentScene(0);
    elapsedTimer1 = 0;
    elapsedTimer2 = 0;
    elapsedTimer3 = 0;
    elapsedTimer4 = 0;

    picoOBJ.begin();
    
    debugserialPrintln(4, "Setup has completed. Continuing to 'loop'.");





    preferences.outPortModes[0] = ext_btn_modes::DualButton;
}

void loop() {
#ifdef DEBUG
    // Look for "esp" on USB serial port. If found, enter ESP programming mode.
    while (USB_Serial.available()) {
        delay(2);
        char inByte = USB_Serial.read();
        if (inByte == 'A' || inByte == 'B')
            ESP8266_Serial.print(inByte);
        if (inByte == 'e') {
            inByte = USB_Serial.read();
            if (inByte == 's') {
                inByte = USB_Serial.read();
                if (inByte == 'p') {
                    inByte = USB_Serial.read();
                    if (inByte == '2') {
                        while (USB_Serial.available())
                            USB_Serial.read();
                        enter_ESP8266_ProgrammingMode(1500000);
                    } else {
                        while (USB_Serial.available())
                            USB_Serial.read();
                        enter_ESP8266_ProgrammingMode(115200);
                    }
                }
            }
        }
    }
#endif

    for (uint8_t i = 0; i < 10; i++) {
        BtnDebouncer[i].update();
        if (BtnDebouncer[i].fell())
            buttonPressed(i);
    }

    if (ESP8266_Serial.peek() != -1)
        handleESP8266Serial();

    // Check all the Midi inputs for data and forward as necessary.

    for (uint8_t i = 0; i < 4; i++) {
        if (HW_midi[i].read() && preferences.passThrough.MIDItoUSB[i]) {
            byte type = HW_midi[i].getType();
            if (type != midi::SystemExclusive)
                usbMIDI.send(type, HW_midi[i].getData1(), HW_midi[i].getData2(), HW_midi[i].getChannel(), 0);
            else
                usbMIDI.sendSysEx((HW_midi[i].getData1() + HW_midi[i].getData2() * 256), HW_midi[i].getSysExArray(), true, 0);
        }
    }


    if (usbMIDI.read()) {
        byte type = usbMIDI.getType();
        byte chan = usbMIDI.getChannel();
        byte data1 = usbMIDI.getData1();
        byte data2 = usbMIDI.getData2();
        byte cable = usbMIDI.getCable();
        if (type != usbMIDI.SystemExclusive) {
            if (preferences.passThrough.USBtoMIDI[cable]) {
                HW_midi[cable].send((midi::MidiType)type, data1, data2, chan);
            }
        } else if (preferences.passThrough.USBtoMIDI[cable])
            HW_midi[cable].sendSysEx(data1 + data2 * 256, usbMIDI.getSysExArray(), true);
    }
    // hasSentTimingInfo = true;
    if (elapsedTimer1 > 200) { // ~ every 100us
        elapsedTimer1 = 0;
        wireBufForLCDs.sendNextOutMessage();
        picoOBJ.update(currentScene);
        for (uint8_t i = 0; i < 10; i++) {
            mainActionQ[i].processQueue(currentScene);
            if (i < 8) {
                extActionQ[i].processQueue(currentScene);
            }
        }
    }

    if (elapsedTimer2 > (950 * preferences.LCD_TextUpdateTime)) { // every 250ms
        elapsedTimer2 = 0;
        LCDS_updateText();
    }

    if (elapsedTimer3 > (950 * 10000)) { // every 1s
        elapsedTimer3 = 0;
    }
}
