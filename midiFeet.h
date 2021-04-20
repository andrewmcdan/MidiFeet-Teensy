#define F_CPU 720000000      // needed here to make VScode play nice.
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
#define dbgserPrint(a) if(5<=DEBUG_LEVEL)Serial.print(a)
#define dbgserPrintln(a) if(5<=DEBUG_LEVEL)Serial.println(a)
#define dbgserPrint_T(a,b) if(5<=DEBUG_LEVEL)Serial.print(a,b)
#define dbgserPrintln_T(a,b) if(5<=DEBUG_LEVEL)Serial.println(a,b)

// USB Serial.println with !!NO!! type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrint(l,a) if(l<=DEBUG_LEVEL)Serial.print(a)
// USB Serial.println with !!NO!! type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintln(l,a) if(l<=DEBUG_LEVEL)Serial.println(a)
// USB Serial.println with type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintType(l,a,p) if(l<=DEBUG_LEVEL)Serial.print(a,p)
// USB Serial.println with type parameter
// l: debug level, higher for lower priority.
// 5-literally anything else, 4-info, 3-debug, 2-warnings, 1-errors, 0-critical
// a: the thing to print
#define debugserialPrintlnType(l,a,p) if(l<=DEBUG_LEVEL)Serial.println(a,p)
#else
#define dbgserPrint(a) nop()
#define dbgserPrintln(a) nop()
#define dbgserPrint_T(a,b) nop()
#define dbgserPrintln_T(a,b) nop()
#define debugSerialBegin(b) nop()
#define debugserialPrint(l,a) nop()
#define debugserialPrintln(l,a) nop()
#define debugserialPrintType(l,a,p) nop()
#define debugserialPrintlnType(l,a,p) nop()
#endif

class RasPiPico;

class PrefsObj
{
private:
public:
    struct PASSTHROUGH_PREFS
    {
        bool USBtoMIDI[4] = {true, true, true, true};
        bool MIDItoUSB[4] = {true, true, true, true};
        byte MIDItoMIDI[4] = {1, 1, 1, 1};
    };

    byte channel[4] = {0, 0, 0, 0};

    byte backlightBrightness = 0;

    enum ArrowPositions : byte
    {
        TopRight_BtmRight = 0x00, // 0b00000000
        TopLeft_BtmRight = 0x01,  // 0b00000001
        TopRight_BtmLeft = 0x02,  // 0b00000010
        TopLeft_BtmLeft = 0x03    // 0b00000011
    };

    ArrowPositions arrowPos;

    uint16_t totalNumberOfScenes,LCD_TextUpdateTime;

    byte expPedalUpdateIntervalPref = 50; // time in ms between updates on the exp pedal.

    PASSTHROUGH_PREFS passThrough;

    PrefsObj(){};
};

enum SERIAL_ERROR : uint8_t
{
    NONE = 0,
    CRC8_MISMATCH = 0x01,
    TIMEOUT = 0x02,
    NO_BYTES_READ = 0x04,
    CRC32_MISMATCH = 0x08,
    SD_FILE_ERROR = 0x03,
};
enum ESP_SERIAL_COMMANDS_Message
{
    RequestForHashCompare = 0x01,
    RequestForPreferences = 0x02,
    RequestForSceneFile = 0x03,
    RequestForOtherFile = 0x04,
    OkToStartSendingData = 0x05,
    OkToContinueSendingData = 0x06,
    RequestForTotalNumberOfScene = 0x07,
    RequestToSaveSceneFile = 0x08,
    isMessageNotPacket = 0xa0,
};
enum ESP_SERIAL_COMMANDS_Packet
{
    StartSendData = 0x10,
    ContinueSendData = 0x20,
    EndSendData = 0x30,
    FileTransfer = 0x01,
    SaveSceneData = 0x02,
    SavePreferences = 0x03,
};
enum ESP_SERIAL_DataType
{
    FileInfo = 0x01, // filename hash, file size,...
    FileNameHash = 0x02,
    FileSize = 0x03,
    FullFileName = 0x04,
    TotalNumOfScenes = 0x05,
};
enum ext_btn_modes : byte
{
    SingleButton = 0x00,
    DualButton = 0x01,
    ExpPedalMinMax = 0x03,
    ExpPedalContinuous = 0x04,
    Disabled = 0xff
};
enum out_port_modes : byte
{
    SingleOutput = 0x00,
    DualOutput = 0x01,
    Disable = 0xff,
    TS_output = SingleOutput,
    TRS_output = DualOutput
};
enum out_port_state : byte
{
    Tip_On = 0x01,
    Ring_On = 0x02,
    TR_On = 0x03,
    Off = 0x00,
    OutPortStateNotSaved = 0xff
};
struct WireBuffer
{
    struct BUF
    {
        byte data[256];
        byte address;
        uint8_t len;
        uint16_t waitTime = 0;
    };
    uint8_t last_in = 0;   // index of the last in data
    uint8_t first_out = 0; // index of the first data to write out
    elapsedMicros microTime;

    BUF buffer[64];

    WireBuffer()
    {
        this->microTime = 0;
    };

    bool addEntryToBuffer(byte newData_addr, byte newData_arr[], uint8_t newData_len, uint16_t newData_wait = 0)
    {
        // Serial.println("Adding entry to wire buffer.");
        if (this->last_in == this->first_out - 1)
        {
            return false; // buffer is full
        }
        this->buffer[this->last_in].address = newData_addr;
        this->buffer[this->last_in].len = newData_len;
        this->buffer[this->last_in].waitTime = newData_wait;
        for (uint8_t i = 0; i < newData_len; i++)
        {
            this->buffer[this->last_in].data[i] = newData_arr[i];
        }
        this->last_in++;
        if (this->last_in == 64)
        {
            this->last_in = 0;
        }
        return true;
    }

    uint8_t sendNextOutMessage()
    {
        // Serial.println("\"Send wire message\" called.");
        if (this->last_in == this->first_out)
        {
            // Serial.println("Wire buffer was empty.");
            return 0;
        }
        // if(micros()>this->lastSentMicros+this->buffer[this->first_out>0?this->first_out-1:63].waitTime){
        if (microTime > this->buffer[this->first_out > 0 ? this->first_out - 1 : 63].waitTime)
        {
            // Serial.println("Sending the message on the \"wire\".");
            Wire.beginTransmission(this->buffer[this->first_out].address);
            for (uint8_t i = 0; i < this->buffer[this->first_out].len; i++)
                Wire.write(this->buffer[this->first_out].data[i]);
            Wire.endTransmission();
            this->first_out++;
            if (this->first_out == 64)
            {
                this->first_out = 0;
            }
            // this->lastSentMicros = micros();
            microTime = 0;
            return this->last_in - this->first_out;
        }
        // Serial.println("data in bufer but timeout not reached.");
        return 0xff;
    }
};
template <size_t Index, class T>
struct NamedArrayElement
{
    char trash;

    operator T &() // allows: double d = object.Member;
    {
        return ((T *)(this))[Index/sizeof(T)];
    }

    T &operator=(T const &rhs) // allows: object.member = 1.0;
    {
        T &me = ((T *)(this))[Index/sizeof(T)];

        me = rhs;

        return me;
    }

    T *operator&() // allows: double *p = &object.Member;
    {
        return &((T *)(this))[Index/sizeof(T)];
    }

    bool operator<(T const &rhs) // allows: if(object.Member < 1.0)
    {
        return ((T *)(this))[Index/sizeof(T)] < rhs;
    }

    bool operator>(T const &rhs) // allows: if(object.Member > 1.0)
    {
        return ((T *)(this))[Index/sizeof(T)] > rhs;
    }

    bool operator==(T const &rhs) // allows: if(object.Member == 1.0)
    {
        return ((T *)(this))[Index/sizeof(T)] == rhs;
    }
};
/// \brief fix me
/// \tparam fix me
/// \param data fix me
/// \param size fix me
template <size_t startIndex, class start_T, int arrSize, class arrSize_T>
struct SpecialNamedElement
{
    char trash;

    arrSize_T &operator=(arrSize_T const &rhs) // allows: object.member = 1.0;
    {
        arrSize_T &me = ((start_T *)(this))[startIndex];

        me = rhs;

        return me;
    }

    arrSize_T &operator[](int const ind)
    {
        return ((arrSize_T *)(&((start_T *)(this))[startIndex]))[ind];
    }
};
/// \brief Serial packet object containing 32bit start sequence (startSequence_32), 32bit crc (crc_32), and 16 bytes of data
struct SerialPacket
{
    union
    {
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
    uint8_t &operator[](unsigned int i) {
        if(i>sizeof(this->full_array)){return this->full_array[0];}
        return this->full_array[i]; 
    }
    
    /// \brief Serial packet object containing 32bit start sequence (startSequence_32), 32bit crc (crc_32), and 8 bytes of data
    SerialPacket()
    {
        for (uint8_t i = 0; i < sizeof(this->full_array); i++)
            this->full_array[i] = i + 1;
    }

    /// \brief Calculate crc32 for startSequence and data, and store it in crc_32 member
    /// \returns 32bit crc
    uint32_t CalculateCRC()
    {
        this->crc_32 = crc32Buffer(this->full_array,(sizeof(this->full_array) - 4));
        return this->crc_32;
        // CRC32 crc;
        // for (size_t i = 0; i < (sizeof(this->full_array) - 4); i++)
        // {
        //     crc.update(this->full_array[i]);
        // }
        // this->crc_32 = crc.finalize();
        // return this->crc_32;
    }
};
#define SHORT_MESSAGE_DATA_LENGTH 4
///\brief Smaller version of SerialPacket.
/// 1 byte start seq, (SHORT_MESSAGE_DATA_LENGTH) 4 bytes data, 1 byte crc.
struct SerialMessage
{
    union
    {
        uint8_t full_array[1+SHORT_MESSAGE_DATA_LENGTH+1];
        NamedArrayElement<0,uint8_t> StartSequence_8;
        NamedArrayElement<1+SHORT_MESSAGE_DATA_LENGTH,uint8_t> crc_8;
        SpecialNamedElement<1,uint8_t,SHORT_MESSAGE_DATA_LENGTH,uint8_t> data;
        // SpecialNamedElement<1,uint8_t,(SHORT_MESSAGE_DATA_LENGTH/4),uint32_t> data_32;
    };
    uint8_t &operator[](int i){return this->full_array[i];}

    ///\brief Smaller version of SerialPacket.
    /// 1 byte start seq, (SHORT_MESSAGE_DATA_LENGTH) 4 bytes data, 1 byte crc.
    SerialMessage(){
        for(uint8_t i=0;i<(sizeof(this->full_array));i++)
            this->full_array[i]=i;
    }
    uint8_t CalculateCRC(){
        this->crc_8 = uint8_t(crc32Buffer(this->full_array,(sizeof(this->full_array) - 1)));
        return this->crc_8;
        // CRC32 crc;
        // for (size_t i = 0; i < (sizeof(this->full_array) - 1); i++)
        // {
        //     crc.update(this->full_array[i]);
        // }
        // this->crc_8 = uint8_t(crc.finalize());
        // return this->crc_8;
    }
};
struct qdAction{
    uint8_t buttonIdNum;
    uint16_t scnNum;
    uint8_t actionNum;
    elapsedMillis elapsedSinceActionCalled = 0;
    unsigned long timeToWait = 0;
    bool hasBeenSent = false;
};
struct buttonActions
{
    enum ActionTypes : byte
    {
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
    void resetToDefaults()
    {
        this->action = ActionTypes::NULL_Action;
        for (int i = 0; i < 16; i++)
        {
            this->actionData[i] = 0;
        }
    }
    ///\brief Do this action's thing. Send some data, toggle a thing, etc.
    bool doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevice,qdAction &actQ, void (*func)(int),PrefsObj &prefs_R, RasPiPico &pico_R, uint16_t currScn=0){
        midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> devArr[] = {midiDevice};
        return doAction(devArr,actQ,func,prefs_R,pico_R,currScn,true);
    }
    ///\brief Do this action's thing. Send some data, toggle a thing, etc.
    bool doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[],qdAction &actQ, void (*func)(int),PrefsObj &prefs_R, RasPiPico &pico_R, uint16_t currScn, bool singleMidi = false)
    {
        actQ.hasBeenSent=true;
        dbgserPrintln("do Action");
        dbgserPrint("this->action: ");
        dbgserPrintln(this->action);
        actQ.elapsedSinceActionCalled = 0; // reset for every action.
        switch (this->action)
        {
        case ActionTypes::NULL_Action:
        {
            return false;
            break;
        }
        case ActionTypes::SendNoteOn:
        {   
            byte midiOutPort = this->actionData[0];
            byte midiChan = this->actionData[1];
            byte noteVal = this->actionData[2];
            byte veloctiy = this->actionData[3];
            midiDevs[midiOutPort].sendNoteOn(noteVal,veloctiy,midiChan);
            break;
        }
        case ActionTypes::SendNoteOff:
        {
            byte midiOutPort = this->actionData[0];
            byte midiChan = this->actionData[1];
            byte noteVal = this->actionData[2];
            byte veloctiy = this->actionData[3];
            midiDevs[midiOutPort].sendNoteOff(noteVal,veloctiy,midiChan);
            break;
        }
        case ActionTypes::SendCC_AbsoluteValue:
        {
            byte midiOutPort = this->actionData[0];
            byte midiChan = this->actionData[1];
            byte midiCC = this->actionData[2];
            byte midiCCval = this->actionData[3];
            midiDevs[midiOutPort].sendControlChange(midiCC,midiCCval,midiChan);
            break;
        }
        case ActionTypes::SendCC_Increment0_127 ... ActionTypes::SendCC_Decrement96_97:
        {
            byte midiOutPort = this->actionData[0];
            byte midiChan = this->actionData[1];
            byte midiCC = this->actionData[2];
            const byte lut[] = {127,0,64,63,97,96};
            byte midiCCval = lut[this->action-ActionTypes::SendCC_Increment0_127];
            midiDevs[midiOutPort].sendControlChange(midiCC,midiCCval,midiChan);
            break;
        }
        case ActionTypes::SendProgChange:
        {
            byte midiOutPort = this->actionData[0];
            byte midiProgNum = this->actionData[1];
            byte midiChan = this->actionData[2];
            midiDevs[midiOutPort].sendProgramChange(midiProgNum,midiChan);
            break;
        }
        case ActionTypes::NextScene:
        {
            // setCurrentScene(currScn+1);
            func(currScn+1);
            break;
        }
        case ActionTypes::PrevScene:
        {
            // setCurrentScene(currScn-1);
            func(currScn-1);
            break;
        }
        case ActionTypes::TurnOnOutPort:
        {
            break;
        }
        case ActionTypes::TurnOffOutPort:
        {
            break;
        }
        case ActionTypes::ToggleOutPort:
        {
            break;
        }
        case ActionTypes::PulseOutPort:
        {
            break;
        }
        case ActionTypes::ActionWait:
        {
            unsigned long minutes = this->actionData[0];
            unsigned long seconds = this->actionData[1];
            unsigned long milliseconds = this->actionData[2];
            actQ.timeToWait = milliseconds + ( seconds * 1000 ) + ( minutes * 60 * 1000);
            dbgserPrintln_T(actQ.timeToWait,HEX);
            break;
        }
        case ActionTypes::JumpToScene:
        {
            uint16_t scnNum = 0;
            if(this->actionData[1]==0){
                scnNum = this->actionData[0];
            }else{
                scnNum = this->actionData[0]<<8;
                scnNum += this->actionData[1];
            }
            // func(scnNum);
            break;
        }
        case ActionTypes::ExpPedalUpdateInterval:
        {
            prefs_R.expPedalUpdateIntervalPref = this->actionData[0];
            break;
        }
        default:
            return false;
            break;
        }
        return true;
    }
};
///\brief A type that can hold 64 unique values. Will rollover from 63 to 0. 
///\param val Init value. Must be less than 64 (or "max" if set)
///\param max Used to set the rollover value to an arbitrary value. 1 to 256.
class countTo64 {
    private:
    uint8_t value;
    uint8_t maxVal;
    public:
    countTo64(void)                                     { value = 0; }
    countTo64(int val)                                  { maxVal = 64; value = val<maxVal?val:0; }
    countTo64(int val, int max)                         { maxVal = (max<257)?max:256; value = val<maxVal?val:0; }
    operator int () const                               { return value; }
    countTo64 & operator = (const countTo64 &rhs)       { value = rhs.value; return *this; }
    countTo64 & operator = (const int &rhs)             { value = rhs; return *this; }
    countTo64 & operator -= (uint8_t val)               { value = (value>=val)?(value-val):maxVal-(val-value) ; return *this; }
	countTo64 & operator += (uint8_t val)               { value = ((value+val)<maxVal)?(value+val):val - (maxVal-value) ; return *this; }
	countTo64 operator - (int val) const                { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (unsigned int val) const       { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (long val) const               { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (uint8_t val) const            { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator + (int val) const                { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (unsigned int val) const       { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (long val) const               { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (uint8_t val) const            { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
    countTo64 operator++ (int val)                      { value = value==(maxVal-1)?0:value+1; return *this; }
    countTo64 operator-- (int val)                      { value = value==0?(maxVal-1):value-1; return *this; }
    bool operator==(const countTo64 &rhs)               { return value == rhs.value; }
    bool operator>=(const countTo64 &rhs)               { return value >= rhs.value; }
    bool operator<=(const countTo64 &rhs)               { return value <= rhs.value; }
    bool operator>(const countTo64 &rhs)                { return value > rhs.value; }
    bool operator<(const countTo64 &rhs)                { return value < rhs.value; }
};
struct MAIN_BTN
{
    char topRowText[50];       // 50 bytes
    buttonActions Actions[32]; // 17 byte each = 17*32 = 544 bytes
    void resetToDefaults()
    {
        char defaultText[] = "Button disabled.";
        for (int i = 0; i < 17; i++)
        {
            this->topRowText[i] = defaultText[i];
        }
        for (int i = 0; i < 32; i++)
        {
            this->Actions[i].resetToDefaults();
        }
    }
};
struct EXT_BTN
{
    buttonActions Actions[32]; // 544 bytes
    ext_btn_modes Btn_Mode;    // 1 byte
    void resetToDefaults()
    {
        this->Btn_Mode = ext_btn_modes::Disabled;
        for (int i = 0; i < 32; i++)
        {
            this->Actions[i].resetToDefaults();
        }
    }
};
struct OUT_PORTS
{
    out_port_modes out_mode; // 1 byte
    out_port_state state;    // 1 byte
    void resetToDefaults()
    {
        this->out_mode = out_port_modes::Disable;
        this->state = out_port_state::OutPortStateNotSaved;
    }
};
struct SCENE // 10308 bytes each
{
    MAIN_BTN mainButtons[10];  // (544 + 50) * 10 = 5940 bytes
    OUT_PORTS output_ports[4]; // 2 * 4 = 8
    EXT_BTN extButtons[8];     // 545 * 8 = 4360 bytes
    void resetToDefaults()
    {
        for (int i = 0; i < 10; i++)
        {
            this->mainButtons[i].resetToDefaults();
        }
        for (int i = 0; i < 4; i++)
        {
            this->output_ports[i].resetToDefaults();
        }
        for (int i = 0; i < 8; i++)
        {
            this->extButtons[i].resetToDefaults();
        }
    }
    SCENE(){};
};
struct SceneObjSingle
{
    MAIN_BTN mainButtons[10];  // (544 + 50) * 10 = 5940 bytes
    OUT_PORTS output_ports[4]; // 2 * 4 = 8
    EXT_BTN extButtons[8];     // 545 * 8 = 4360 bytes
    void resetToDefaults()
    {
        for (int i = 0; i < 10; i++)
        {
            this->mainButtons[i].resetToDefaults();
        }
        for (int i = 0; i < 4; i++)
        {
            this->output_ports[i].resetToDefaults();
        }
        for (int i = 0; i < 8; i++)
        {
            this->extButtons[i].resetToDefaults();
        }
    }
    // When the constructor is called, this will allocate enough memory to hold MAX_NUMBER_OF_SCENES.
    // the member "numScenes" exists to allow fast initalization of the memory space when fewer
    // than MAX_NUMBER_OF_SCENES is allocated. Strictly speaking, all of the scenes that are allocated
    // can be used, they just won't be init'd with the initValues func.

    SceneObjSingle(){};

    void initValues()
    {
        for (int u = 0; u < 8; u++)
        {
            for (int p = 0; p < 32; p++)
            {
                this->extButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                for (int q = 0; q < 16; q++)
                {
                    this->extButtons[u].Actions[p].actionData[q] = 0;
                }
            }
            this->extButtons[u].Btn_Mode = ext_btn_modes::Disabled;
        }
        for (int u = 0; u < 4; u++)
        {
            this->output_ports[u].out_mode = out_port_modes::DualOutput;
            this->output_ports[u].state = out_port_state::Off;
        }
        for (int u = 0; u < 10; u++)
        {
            for (int p = 0; p < 32; p++)
            {
                this->mainButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                for (int q = 0; q < 16; q++)
                {
                    this->mainButtons[u].Actions[p].actionData[q] = 0;
                }
            }
            for (int p = 0; p < 50; p++)
            {
                this->mainButtons[u].topRowText[p] = '\0';
            }
        }
    };
};
struct SceneObj
{
    int numScenes = 0;

    SCENE scenesArr[MAX_NUMBER_OF_SCENES];
    // When the constructor is called, this will allocate enough memory to hold MAX_NUMBER_OF_SCENES.
    // the member "numScenes" exists to allow fast initalization of the memory space when fewer
    // than MAX_NUMBER_OF_SCENES is allocated. Strictly speaking, all of the scenes that are allocated
    // can be used, they just won't be init'd with the initValues func.

    SceneObj()
    {
        this->numScenes = MAX_NUMBER_OF_SCENES;
    };
    SceneObj(int numberOfScenes)
    {
        this->numScenes = numberOfScenes;
    };

    void initValues()
    {
        for (int i = 0; i < this->numScenes; i++)
        {
            for (int u = 0; u < 8; u++)
            {
                for (int p = 0; p < 32; p++)
                {
                    this->scenesArr[i].extButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                    for (int q = 0; q < 16; q++)
                    {
                        this->scenesArr[i].extButtons[u].Actions[p].actionData[q] = 0;
                    }
                }
                this->scenesArr[i].extButtons[u].Btn_Mode = ext_btn_modes::Disabled;
            }
            for (int u = 0; u < 4; u++)
            {
                this->scenesArr[i].output_ports[u].out_mode = out_port_modes::DualOutput;
                this->scenesArr[i].output_ports[u].state = out_port_state::Off;
            }
            for (int u = 0; u < 10; u++)
            {
                for (int p = 0; p < 32; p++)
                {
                    this->scenesArr[i].mainButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                    for (int q = 0; q < 16; q++)
                    {
                        this->scenesArr[i].mainButtons[u].Actions[p].actionData[q] = 0;
                    }
                }
                for (int p = 0; p < 50; p++)
                {
                    this->scenesArr[i].mainButtons[u].topRowText[p] = '\0';
                }
            }
        }
    };
};
///\brief FIFO for actions.
struct buttonActionQueue{
    countTo64 firstOut = 0;
    countTo64 actionsInQ = 0;
    countTo64 lastIn = 0;
    qdAction actionsQ[64];
    bool isMainButtonQueue;
    SceneObj *scenes;
    midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> *midiDevs;
    void(*func)(int);
    PrefsObj *pref_R;
    RasPiPico *pico_obj_P;
    unsigned long startTime;
    ///\brief INIT
    ///\param isMain Must be set according to which buttons within scenes array to access.
    ///\param scenes Must pass the global SceneObj to this constructor.
    ///\param midiDevs Must pass array of midi devices
    ///\param func Function to call when changing current scene
    ///\param pref_R Must pass the global PrefsObj
    buttonActionQueue(bool isMain,SceneObj &scenes, midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[], void(*func)(int), PrefsObj &pref_R){
        this->isMainButtonQueue=isMain;
        this->pref_R=&pref_R;
        this->scenes=&scenes;
        this->func=func;
        this->midiDevs = midiDevs;
    }

    void setPicoObj(RasPiPico &pico_R){
        this->pico_obj_P=&pico_R;
    }
    
    int addAction(uint8_t btn, uint8_t num, uint16_t scene){
        if(this->lastIn == this->firstOut - 1)
            return 0; // queue is full
        this->actionsQ[this->lastIn].actionNum = num;
        this->actionsQ[this->lastIn].buttonIdNum = btn;
        this->actionsQ[this->lastIn].scnNum = scene;
        this->actionsQ[this->lastIn].hasBeenSent = false;
        this->lastIn++;
        return this->lastIn - this->firstOut;
    }

    int processQueue(uint16_t currScn = 0xffff){
        if(this->firstOut == this->lastIn){
            //queue is empty
            return 0;
        }
        uint8_t btn = this->actionsQ[this->firstOut].buttonIdNum;
        uint8_t act = this->actionsQ[this->firstOut].actionNum;
        uint16_t scn = this->actionsQ[this->firstOut].scnNum;
        
        // check to see if this act is a "wait action"
        // if so, check elapsed time since relavent action (first or previous)
        bool isWait;
        if(this->isMainButtonQueue){
            
            isWait = (buttonActions::ActionTypes::ActionWait == scenes->scenesArr[scn].mainButtons[btn].Actions[act].action);
        }else{
            isWait = (buttonActions::ActionTypes::ActionWait == scenes->scenesArr[scn].extButtons[btn].Actions[act].action);
        }
        if(isWait){
            // actionData[15] shall hold a value to determine if the wait has started.
            bool isSent = this->actionsQ[this->firstOut].hasBeenSent;
            // if this is first time we've been here, call doAction() for the wait action to set the time it was called.
            if(!isSent){
                if(this->isMainButtonQueue){
                    dbgserPrintln("isFirst was true.");
                    scenes->scenesArr[scn].mainButtons[btn].Actions[act].doAction(midiDevs,this->actionsQ[this->firstOut],func,*pref_R,*pico_obj_P,currScn);
                }else{
                    scenes->scenesArr[scn].extButtons[btn].Actions[act].doAction(midiDevs,this->actionsQ[this->firstOut],func,*pref_R,*pico_obj_P,currScn);
                }
            }
            if(this->actionsQ[this->firstOut].elapsedSinceActionCalled > (this->actionsQ[this->firstOut].timeToWait * 0.95)){
                this->firstOut++;
            }
            return this->lastIn - this->firstOut;
        }
        
        // if it was not a wait action, call doAction for this action.
        if(this->isMainButtonQueue){
            scenes->scenesArr[scn].mainButtons[btn].Actions[act].doAction(midiDevs,this->actionsQ[this->firstOut], func, *pref_R,*pico_obj_P,currScn);
        }else{
            scenes->scenesArr[scn].extButtons[btn].Actions[act].doAction(midiDevs,this->actionsQ[this->firstOut], func, *pref_R,*pico_obj_P,currScn);
        }
        this->firstOut++;
        return this->lastIn - this->firstOut;
    }
};

class OutputPortControl{
    private:
    uint8_t portNum;
    uint8_t unSpecDNum = 0;
    uint8_t portState = 0xaa;
    public:
    OutputPortControl(){}
    OutputPortControl(uint8_t num){
        this->portNum=num;
    }
    bool changePending = false;
    bool pulseInProgress = false;
    elapsedMillis timer;
    unsigned long pulseTime;
    void toggle(){
        if(this->portState==0){
            this->turnOn();
        }else{
            this->turnOff();
        }
    }
    void turnOn(){
        this->portState=1;
    }
    void turnOff(){
        this->portState=0;
    }
    uint8_t OR_state(){
        return portState!=0xaa?(portState<<portNum):0;
    }
};

enum extPedalMode{
    TRS_this_T,
    TRS_this_R,
    TS_this_T,
    EXPRESSION
};

struct ExtPedalState{
    extPedalMode mode;
    bool fallingEdgeEvent;
    bool risingEdgeEvent;
    bool state_OPEN;
    bool state_CLOSED;
};

class ExtPedalInput{
    private:
    ExtPedalState state_T;
    ExtPedalState state_R;
    public:
    ExtPedalInput(){}
};

class RasPiPico{
    private:
    uint8_t address;
    OutputPortControl outputPort[8];
    elapsedMillis updateIntervalTimer = 0;
    PrefsObj *pref;
    buttonActionQueue *actQ_P;
    SceneObj *scenes;
    enum picoWireCommands{
        OutputStateUpdate,
        SetOutputMode,
        SetInputMode,
        SetInputUpdateRate,
        RequestUpdate = 0b10000000
    };
    ExtPedalInput extIns[4];
    uint16_t currScn=0;
    public:
    RasPiPico(){}
    RasPiPico(uint8_t addr, SceneObj &scenes, PrefsObj &pref_R, buttonActionQueue actQ_R[]){
        this->address = addr;
        for(uint8_t i=0;i<8;i++){
            this->outputPort[i] = OutputPortControl(i);
        }
        this->pref = &pref_R;
        this->actQ_P = actQ_R;
        this->scenes = &scenes;      
    }
    void toggleOutput(uint8_t port_num){
        this->outputPort[port_num].toggle();
    }
    void setOutputOn(uint8_t port_num){
        this->outputPort[port_num].turnOn();
    }
    void setOutputOff(uint8_t port_num){
        this->outputPort[port_num].turnOff();
    }
    void pulseOutput(uint8_t port_num, unsigned long time){
        this->outputPort[port_num].pulseInProgress = true;
        this->outputPort[port_num].pulseTime = time;
        this->outputPort[port_num].timer = 0;
        toggleOutput(port_num);
    }
    void setInputConfig(uint16_t sceneNum){

    }
    void update(uint16_t currScn){
        this->currScn=currScn;
        if(this->updateIntervalTimer>pref->expPedalUpdateIntervalPref){
            this->updateIntervalTimer=0;
            // get update from PICO, and store values into private members
            int numReadByte = 0;
            byte readBytes[4] = {0, 0, 0, 0};
            uint8_t iterator = 0;
            // numReadByte = Wire.requestFrom(ADDR_I2C_TO_EXT_BTN_CONTROLLER,4);
            // numReadByte = numReadByte<5?numReadByte:4;
            // for(uint8_t i=0;i<numReadByte;i++){
            //     readBytes[i] = Wire.read();
            // }

            Wire.beginTransmission(ADDR_I2C_TO_EXT_BTN_CONTROLLER);
            Wire.write(0);
            Wire.write(0xaa);
            Wire.write(0xaa);
            // Wire.write(0xaa);
            // Wire.write(0xaa);
            // Wire.write(0xaa);
            Wire.endTransmission();
            
            // Wire.requestFrom(ADDR_I2C_TO_EXT_BTN_CONTROLLER,4);
            // Wire.readBytes(readBytes,4);
            
            // dbgserPrint_T(readBytes[0],HEX);
            // dbgserPrint_T(readBytes[1],HEX);
            // dbgserPrint_T(readBytes[2],HEX);
            // dbgserPrint_T(readBytes[3],HEX);
            // dbgserPrintln("");
            // ext_btn_modes::

            // If an event has happened, add action to queue.
            if(readBytes[0]>0){
            }
        }

        // check to see if any pulses are in progress, and if so, check to see if it has expired.
        for(uint8_t i=0;i<8;i++){
            if(this->outputPort[i].pulseInProgress && (this->outputPort[i].timer > this->outputPort[i].pulseTime)){
                toggleOutput(i);
            }
        }
        // check to see if any state changes are pending.
        // have to use seperate BOOL because pending does not necessarily indicate that a change is pending. It will indicate the requested states.
        bool isAnyPending = false;
        uint8_t pending = 0;
        for(uint8_t i=0;i<8;i++){
            if(this->outputPort[i].changePending){
                isAnyPending=true;
                pending |= this->outputPort[i].OR_state();
            }
        }
        if(isAnyPending){
            // send update to Pico on wire
            // 2 byte;
            //       first is command "picoWireCommands::OutputStateUpdate" (0x00)
            //       second is requested states of all outputs
            Wire.beginTransmission(ADDR_I2C_TO_EXT_BTN_CONTROLLER);
            Wire.write(picoWireCommands::OutputStateUpdate);
            Wire.write(pending);
            Wire.endTransmission();
        }
    }
};
