// ignores most of the code
// cSpell:ignoreRegExp /(^(?!\s*(\/\/)|(\/\*)).*[;:)}=,{])/gm

// ignores any word in quotes
// cSpell:ignoreRegExp /\"\S*\"/g

//--- ignores HEX literals
// cSpell:ignoreRegExp /0x[A-Z]+/g

//--- ignores any preprocessor directive (i.e #define)
// cSpell:ignoreRegExp /(^#.*)/gm

/// words to ignore
// cSpell:ignore pico PSRAM btn btns spec'd dbgserPrintln dbgser Println devs

/// spell check extension defaults to checking each part of camel case words as separate words.

#ifndef DEBUG
#define DEBUG
#endif

#define F_CPU 720000000 // needed here to make VScode play nice.
#ifndef MAX_NUMBER_OF_SCENES
#define MAX_NUMBER_OF_SCENES 1500
#endif
#include <Wire.h>
#include <MIDI.h>
#include <ErriezCRC32.h>
#define ADDR_I2C_TO_LCD 0x6c
#define ADDR_I2C_TO_EXT_BTN_CONTROLLER 0x5a
#define LCD_COMMAND_SET_COLS_ROWS 0x05
#define LCD_COMMAND_START_LCD 0x0b
#define LCD_COMMAND_MK_CUST_CHAR_BASE 0x20
#define LCD_COMMAND_SEND_CUST_CHAR 0x30
#define LCD_COMMAND_DISP_TEXT_BASE 0xa0
#define LCD_COMMAND_SET_BL_BRIGHTNESS 0xff
#define ESP8266_Serial Serial8
#define USB_Serial Serial
#define ESP_RESET 31
#define ESP_BOOT_MODE 32
#ifdef DEBUG
#define debugSerialBegin(b) Serial.begin(b)
#define dbgserPrint(a) if (5 <= DEBUG_LEVEL) Serial.print(a)
#define dbgserPrintln(a) if (5 <= DEBUG_LEVEL) Serial.println(a)
#define dbgserPrint_T(a, b) if (5 <= DEBUG_LEVEL) Serial.print(a, b)
#define dbgserPrintln_T(a, b) if (5 <= DEBUG_LEVEL) Serial.println(a, b)
// USB Serial.println with !!NO!! type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrint(l, a) if (l <= DEBUG_LEVEL) Serial.print(a)
// USB Serial.println with !!NO!! type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintln(l, a) if (l <= DEBUG_LEVEL) Serial.println(a)
// USB Serial.println with type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintType(l, a, p) if (l <= DEBUG_LEVEL) Serial.print(a, p)
// USB Serial.println with type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintlnType(l, a, p) if (l <= DEBUG_LEVEL) Serial.println(a, p)
#else
#define dbgserPrint(a) nop()
#define dbgserPrintln(a) nop()
#define dbgserPrint_T(a, b) nop()
#define dbgserPrintln_T(a, b) nop()
#define debugSerialBegin(b) nop()
#define debugserialPrint(l, a) nop()
#define debugserialPrintln(l, a) nop()
#define debugserialPrintType(l, a, p) nop()
#define debugserialPrintlnType(l, a, p) nop()
#endif
#define SHORT_MESSAGE_DATA_LENGTH 4

/// \brief Do literally nothing. Exists for debugging purposes.
/// Compiler will almost certainly remove any call to this function.
// void nop() {}

enum extPedalMode {
    TRS_this_T,
    TRS_this_R,
    TS_this_T,
    EXPRESSION
};
enum out_port_state : byte {
    Tip_On = 0x01,
    Ring_On = 0x02,
    TR_On = 0x03,
    Off = 0x00,
    OutPortStateNotSaved = 0xff
};
enum SERIAL_ERROR : uint8_t {
    NONE = 0,
    CRC8_MISMATCH = 0x01,
    TIMEOUT = 0x02,
    NO_BYTES_READ = 0x04,
    CRC32_MISMATCH = 0x08,
    SD_FILE_ERROR = 0x03,
};
enum ESP_SERIAL_COMMANDS_Message {
    RequestForHashCompare = 0x01,
    RequestForPreferences = 0x02,
    RequestForSceneFile = 0x03,
    RequestForOtherFile = 0x04,
    OkToStartSendingData = 0x05,
    OkToContinueSendingData = 0x06,
    RequestForTotalNumberOfScene = 0x07,
    RequestToSaveSceneFile = 0x08,
    RequestToSavePrefsFile = 0x09,
    isMessageNotPacket = 0xa0,
};
enum ESP_SERIAL_COMMANDS_Packet {
    StartSendData = 0x10,
    ContinueSendData = 0x20,
    EndSendData = 0x30,
    FileTransfer = 0x01,
    SaveSceneData = 0x02,
    SavePreferences = 0x03,
};
enum ESP_SERIAL_DataType {
    FileInfo = 0x01, // filename hash, file size,...
    FileNameHash = 0x02,
    FileSize = 0x03,
    FullFileName = 0x04,
    TotalNumOfScenes = 0x05,
};
enum ext_btn_modes : byte {
    SingleButton = 0x00,
    DualButton = 0x01,
    ExpPedalMinMax = 0x03,
    ExpPedalContinuous = 0x04,
    Disabled = 0xff
};
enum out_port_modes : byte {
    SingleOutput = 0x00,
    DualOutput = 0x01,
    Disable = 0xff,
    TS_output = SingleOutput,
    TRS_output = DualOutput
};
template <size_t Index, class T>
struct NamedArrayElement {
    char trash;
    operator T& () { return ((T*)(this))[Index / sizeof(T)]; }                                      // allows: double d = object.Member;
    T& operator=(T const& rhs) { T& me = ((T*)(this))[Index / sizeof(T)];me = rhs;return me; }      // allows: object.member = 1.0;
    T* operator&() { return &((T*)(this))[Index / sizeof(T)]; }                                     // allows: double *p = &object.Member;
    bool operator<(T const& rhs) { return ((T*)(this))[Index / sizeof(T)] < rhs; }                  // allows: if(object.Member < 1.0)
    bool operator>(T const& rhs) { return ((T*)(this))[Index / sizeof(T)] > rhs; }                  // allows: if(object.Member > 1.0)
    bool operator==(T const& rhs) { return ((T*)(this))[Index / sizeof(T)] == rhs; }                // allows: if(object.Member == 1.0)
};
template <size_t startIndex, class start_T, int arrSize, class arrSize_T>
struct SpecialNamedElement {
    char trash;
    arrSize_T& operator=(arrSize_T const& rhs) { arrSize_T& me = ((start_T*)(this))[startIndex];me = rhs;return me; } // allows: object.member = 1.0;
    arrSize_T& operator[](int const ind) { return ((arrSize_T*)(&((start_T*)(this))[startIndex]))[ind]; }
};

struct PrefsObj;
struct WireBuffer;
struct SerialPacket;
struct SerialMessage;
struct qdAction;
class OutputPortControl;
struct ExpPedalState;
class ExpPedalInput;
class countTo64;
class RasPiPico;
struct buttonActions;
struct MAIN_BTN;
struct EXT_BTN;
struct OUT_PORTS;
struct SCENE;
struct SceneObjSingle;
struct SceneObj;
struct ButtonActionQueue;

struct PrefsObj {
    struct PASSTHROUGH_PREFS {
        bool USBtoMIDI[4] = { true, true, true, true };
        bool MIDItoUSB[4] = { true, true, true, true };
        byte MIDItoMIDI[4] = { 1, 1, 1, 1 };
    };
    byte channel[4] = { 0, 0, 0, 0 };
    byte backlightBrightness = 0;
    enum ArrowPositions : byte {
        TopRight_BtmRight = 0x00, // 0b00000000
        TopLeft_BtmRight = 0x01,  // 0b00000001
        TopRight_BtmLeft = 0x02,  // 0b00000010
        TopLeft_BtmLeft = 0x03    // 0b00000011
    };
    ArrowPositions arrowPos;
    uint16_t totalNumberOfScenes, LCD_TextUpdateTime;
    byte expPedalUpdateIntervalPref = 50; // time in ms between updates on the exp pedal.
    PASSTHROUGH_PREFS passThrough;
    uint8_t PortModes[4] = {
        out_port_modes::Disable,
        out_port_modes::Disable,
        out_port_modes::Disable,
        out_port_modes::Disable,
    };
    PrefsObj() {};
};
struct WireBuffer {
    struct BUF {
        byte data[256];
        byte address;
        uint8_t len;
        uint16_t waitTime = 0;
    };
    uint8_t last_in = 0;   // index of the last in data
    uint8_t first_out = 0; // index of the first data to write out
    elapsedMicros microTime;
    BUF buffer[64];
    WireBuffer();
    bool addEntryToBuffer(byte newData_addr, byte newData_arr[], uint8_t newData_len, uint16_t newData_wait = 0);
    uint8_t sendNextOutMessage();
};
/// \brief Serial packet object containing 32bit start sequence (startSequence_32), 32bit crc (crc_32), and 16 bytes of data
struct SerialPacket {
    union {
        // byte array of packet.
        // 4 bytes of start sequence (2 bytes) + command (2 bytes), 16 bytes of data, and 4 bytes for crc.
        // DO NOT ACCESS DIRECTLY! Use object[] interface.
        uint8_t full_array[4 + 16 + 4];
        // uint32_t start sequence and command
        // first 2 bytes are start sequence (0x55, 0xaa)
        // last 2 bytes are command
        NamedArrayElement<0, uint32_t> startSequence_32;
        // uint32_t crc32 of start sequence + command + data
        NamedArrayElement<4 + 16, uint32_t> crc_32;
        // 32bit data array. 4 X uint32_t = 16 bytes.
        SpecialNamedElement<4, uint8_t, 4, uint32_t> data_32;
        // 8bit data array. 16 X uint8_t = 16 bytes.
        SpecialNamedElement<4, uint8_t, 16, uint8_t> data;
    };
    uint8_t& operator[](unsigned int i) {
        if (i > sizeof(this->full_array)) {
            return this->full_array[0];
        }
        return this->full_array[i];
    }
    /// \brief Serial packet object containing 32bit start sequence (startSequence_32), 32bit crc (crc_32), and 8 bytes of data
    SerialPacket();
    /// \brief Calculate crc32 for startSequence and data, and store it in crc_32 member
    /// \returns 32bit crc
    uint32_t CalculateCRC();
};
///\brief Smaller version of SerialPacket.
/// 1 byte start seq, (SHORT_MESSAGE_DATA_LENGTH) 4 bytes data, 1 byte crc.
struct SerialMessage {
    union {
        uint8_t full_array[1 + SHORT_MESSAGE_DATA_LENGTH + 1];
        NamedArrayElement<0, uint8_t> StartSequence_8;
        NamedArrayElement<1 + SHORT_MESSAGE_DATA_LENGTH, uint8_t> crc_8;
        SpecialNamedElement<1, uint8_t, SHORT_MESSAGE_DATA_LENGTH, uint8_t> data;
        // SpecialNamedElement<1,uint8_t,(SHORT_MESSAGE_DATA_LENGTH/4),uint32_t> data_32;
    };
    uint8_t& operator[](int i) { return this->full_array[i]; }
    ///\brief Smaller version of SerialPacket.
    /// 1 byte start seq, (SHORT_MESSAGE_DATA_LENGTH) 4 bytes data, 1 byte crc.
    SerialMessage();
    uint8_t CalculateCRC();
};
struct qdAction {
    uint8_t buttonIdNum;
    uint16_t scnNum;
    uint8_t actionNum;
    elapsedMillis elapsedSinceActionCalled = 0;
    unsigned long timeToWait = 0;
    bool hasBeenSent = false;
};
class OutputPortControl {
private:
    uint8_t portNum;
    uint8_t unSpecDNum = 0;
    uint8_t portState = 0xaa;

public:
    // OutputPortControl(){}
    OutputPortControl(uint8_t num);
    bool changePending = false;
    bool pulseInProgress = false;
    elapsedMillis timer;
    unsigned long pulseTime;
    void toggle();
    void turnOn();
    void turnOff();
    uint8_t OR_state();
};
struct ExtPedalState {
    extPedalMode mode;
    bool fallingEdgeEvent;
    bool risingEdgeEvent;
    bool state_OPEN;
    bool state_CLOSED;
};
class ExtPedalInput {
private:
    ExtPedalState state_T;
    ExtPedalState state_R;
    // uint8_t portMode = ext_btn_modes::Disabled;
public:
    ExtPedalInput(){};
    // uint8_t mode(){
    //     return this->portMode;
    // }
};
class countTo64 {
private:
    uint8_t value;
    uint8_t maxVal;
public:
    countTo64(void) { value = 0; }
    countTo64(int val) { maxVal = 64; value = val < maxVal ? val : 0; }
    countTo64(int val, int max) { maxVal = (max < 257) ? max : 256; value = val < maxVal ? val : 0; }
    operator int() const { return value; }
    countTo64& operator = (const countTo64& rhs) { value = rhs.value; return *this; }
    countTo64& operator = (const int& rhs) { value = rhs; return *this; }
    countTo64& operator -= (uint8_t val) { value = (value >= val) ? (value - val) : maxVal - (val - value); return *this; }
    countTo64& operator += (uint8_t val) { value = ((value + val) < maxVal) ? (value + val) : val - (maxVal - value); return *this; }
    countTo64 operator - (int val) const { countTo64 r(*this); r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value); return r; }
    countTo64 operator - (unsigned int val) const { countTo64 r(*this); r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value); return r; }
    countTo64 operator - (long val) const { countTo64 r(*this); r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value); return r; }
    countTo64 operator - (uint8_t val) const { countTo64 r(*this); r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value); return r; }
    countTo64 operator + (int val) const { countTo64 r(*this); r.value = ((r.value + val) < r.maxVal) ? (r.value + val) : val - (r.maxVal - r.value); return r; }
    countTo64 operator + (unsigned int val) const { countTo64 r(*this); r.value = ((r.value + val) < r.maxVal) ? (r.value + val) : val - (r.maxVal - r.value); return r; }
    countTo64 operator + (long val) const { countTo64 r(*this); r.value = ((r.value + val) < r.maxVal) ? (r.value + val) : val - (r.maxVal - r.value); return r; }
    countTo64 operator + (uint8_t val) const { countTo64 r(*this); r.value = ((r.value + val) < r.maxVal) ? (r.value + val) : val - (r.maxVal - r.value); return r; }
    countTo64 operator++ (int val) { value = value == (maxVal - 1) ? 0 : value + 1; return *this; }
    countTo64 operator-- (int val) { value = value == 0 ? (maxVal - 1) : value - 1; return *this; }
    bool operator==(const countTo64& rhs) { return value == rhs.value; }
    bool operator==(const int& rhs) { return value == rhs; }
    bool operator==(const unsigned int& rhs) { return value == rhs; }
    bool operator>=(const countTo64& rhs) { return value >= rhs.value; }
    bool operator<=(const countTo64& rhs) { return value <= rhs.value; }
    bool operator>(const countTo64& rhs) { return value > rhs.value; }
    bool operator<(const countTo64& rhs) { return value < rhs.value; }
};
class RasPiPico {
private:
    uint8_t address;
    OutputPortControl outputPort[8] = {
        OutputPortControl(0),
        OutputPortControl(1),
        OutputPortControl(2),
        OutputPortControl(3),
        OutputPortControl(4),
        OutputPortControl(5),
        OutputPortControl(6),
        OutputPortControl(7),
    };
    elapsedMillis updateIntervalTimer = 0;
    PrefsObj* pref;
    enum picoWireCommands {
        OutputStateUpdate,
        SetOutputMode,
        SetInputMode,
        SetInputUpdateRate,
        RequestUpdate = 0b10000000
    };
    ExtPedalInput extIns[4];
    uint16_t currScn = 0;

public:
    RasPiPico();
    RasPiPico(uint8_t addr, PrefsObj& pref_R);
    void toggleOutput(uint8_t port_num);
    void setOutputOn(uint8_t port_num);
    void setOutputOff(uint8_t port_num);
    void pulseOutput(uint8_t port_num, uint8_t time);
    void setInputConfig(uint16_t sceneNum);
    void update(uint16_t currScn);
};
struct buttonActions {
    enum ActionTypes : byte {
        SendNoteOn = 0x00,
        SendNoteOff = 0x01,
        SendCC_AbsoluteValue = 0x02,
        SendCC_Increment0_127 = 0x03,
        SendCC_Decrement0_127 = 0x04,
        SendCC_Increment63_64 = 0x05,
        SendCC_Decrement63_64 = 0x06,
        SendCC_Increment96_97 = 0x07,
        SendCC_Decrement96_97 = 0x08,
        SendProgChange = 0x09,
        NextScene = 0x0a,
        PrevScene = 0x0b,
        TurnOnOutPort = 0x0c,
        TurnOffOutPort = 0x0d,
        ToggleOutPort = 0x0e,
        PulseOutPort = 0x0f,
        ActionWait = 0x10,
        JumpToScene = 0x11,
        ExpPedalUpdateInterval = 0x12,
        NULL_Action = 0xff // end of actions
    };
    ActionTypes action;
    byte actionData[16];
    void resetToDefaults();
    bool doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevice, qdAction& actQ, void (*SceneChange_F)(int), PrefsObj& prefs_R, RasPiPico& pico_R, uint16_t currScn = 0);
    bool doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[], qdAction& actQ, void (*SceneChange_F)(int), PrefsObj& prefs_R, RasPiPico& pico_R, uint16_t currScn, bool singleMidi = false);
};
struct MAIN_BTN {
    char topRowText[50];       // 50 bytes
    buttonActions Actions[32]; // 17 byte each = 17*32 = 544 bytes
    void resetToDefaults();
};
struct EXT_BTN {
    buttonActions Actions[32]; // 544 bytes
    ext_btn_modes Btn_Mode;    // 1 byte
    void resetToDefaults();
};
struct OUT_PORTS {
    out_port_modes out_mode; // 1 byte
    out_port_state state;    // 1 byte
    void resetToDefaults();
};
struct SCENE { // 10308 bytes each
    MAIN_BTN mainButtons[10];  // (544 + 50) * 10 = 5940 bytes
    OUT_PORTS output_ports[4]; // 2 * 4 = 8
    EXT_BTN extButtons[8];     // 545 * 8 = 4360 bytes
    void resetToDefaults();
    SCENE() {};
};
struct SceneObjSingle {
    MAIN_BTN mainButtons[10];  // (544 + 50) * 10 = 5940 bytes
    OUT_PORTS output_ports[4]; // 2 * 4 = 8
    EXT_BTN extButtons[8];     // 545 * 8 = 4360 bytes
    void resetToDefaults();
    SceneObjSingle() {};
    void initValues();
};
struct SceneObj {
    int numScenes = 0;
    SCENE scenesArr[MAX_NUMBER_OF_SCENES];
    SceneObj();
    SceneObj(int numberOfScenes);
    void initValues();
};
///\brief FIFO for actions.
struct buttonActionQueue {
    countTo64 firstOut = 0;
    countTo64 actionsInQ = 0;
    countTo64 lastIn = 0;
    qdAction actionsQ[64];
    bool isMainButtonQueue;
    SceneObj* scenes;
    midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>* midiDevs;
    void (*SceneChange_F)(int);
    PrefsObj* pref_R;
    RasPiPico* pico_obj_P;
    unsigned long startTime;
    buttonActionQueue(bool isMain, SceneObj& scenes, midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[], void (*SceneChange_F)(int), PrefsObj& pref_R);
    void setPicoObj(RasPiPico& pico_R);
    int addAction(uint8_t btn, uint8_t num, uint16_t scene);
    int processQueue(uint16_t currScn = 0xffff);
};