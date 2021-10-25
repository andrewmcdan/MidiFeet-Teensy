/*#region*/
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
/*#endregion*/
#include "midiFt_lib.h"
#include "Arduino.h"

WireBuffer::WireBuffer() {
    this->microTime = 0;
};

bool WireBuffer::addEntryToBuffer(byte newData_addr, byte newData_arr[], uint8_t newData_len, uint16_t newData_wait) {
    // Serial.println("Adding entry to wire buffer.");
    if (this->last_in == this->first_out - 1) {
        return false; // buffer is full
    }
    this->buffer[this->last_in].address = newData_addr;
    this->buffer[this->last_in].len = newData_len;
    this->buffer[this->last_in].waitTime = newData_wait;
    for (uint8_t i = 0; i < newData_len; i++) {
        this->buffer[this->last_in].data[i] = newData_arr[i];
    }
    this->last_in++;
    if (this->last_in == 64) {
        this->last_in = 0;
    }
    return true;
}

uint8_t WireBuffer::sendNextOutMessage() {
    if (this->last_in == this->first_out) return 0;
    if (this->microTime > this->buffer[this->first_out > 0 ? this->first_out - 1 : 63].waitTime) {
        Wire.beginTransmission(this->buffer[this->first_out].address);
        for (uint8_t i = 0; i < this->buffer[this->first_out].len; i++)
            Wire.write(this->buffer[this->first_out].data[i]);
        Wire.endTransmission();
        this->first_out++;
        if (this->first_out == 64) {
            this->first_out = 0;
        }
        this->microTime = 0;
        return this->last_in - this->first_out;
    }
    return 0xff;
}

/// \brief Serial packet object containing 32bit start sequence (startSequence_32), 32bit crc (crc_32), and 8 bytes of data
SerialPacket::SerialPacket() {
    for (uint8_t i = 0; i < sizeof(this->full_array); i++)
        this->full_array[i] = i + 1;
}

/// \brief Calculate crc32 for startSequence and data, and store it in crc_32 member
/// \returns 32bit crc
uint32_t SerialPacket::CalculateCRC() {
    this->crc_32 = crc32Buffer(this->full_array, (sizeof(this->full_array) - 4));
    return this->crc_32;
}

SerialMessage::SerialMessage() {
    for (uint8_t i = 0; i < (sizeof(this->full_array)); i++) this->full_array[i] = i;
}

uint8_t SerialMessage::CalculateCRC() {
    this->crc_8 = uint8_t(crc32Buffer(this->full_array, (sizeof(this->full_array) - 1)));
    return this->crc_8;
}

OutputPortControl::OutputPortControl(uint8_t num) {
    this->portNum = num;
}

void OutputPortControl::toggle() {
    if (this->portState == 0) {
        this->turnOn();
    } else {
        this->turnOff();
    }
}
void OutputPortControl::turnOn() {
    this->changePending = true;
    this->portState = 1;
}
void OutputPortControl::turnOff() {
    this->changePending = true;
    this->portState = 0;
}
uint8_t OutputPortControl::OR_state() {
    return this->portState != 0xaa ? (this->portState << this->portNum) : 0;
}

void (RasPiPico::* qhandler_func)(uint8_t, uint8_t, uint16_t);
RasPiPico::RasPiPico() {}
RasPiPico::RasPiPico(uint8_t addr, PrefsObj& pref_R, void (*queueHandlerFunc)(uint8_t, uint8_t, uint8_t)) {
    this->address = addr;
    this->pref = &pref_R;
    this->qhandler_func = queueHandlerFunc;
}
void RasPiPico::toggleOutput(uint8_t port_num) {
    this->outputPort[port_num].toggle();
}
void RasPiPico::setOutputOn(uint8_t port_num) {
    this->outputPort[port_num].turnOn();
}
void RasPiPico::setOutputOff(uint8_t port_num) {
    this->outputPort[port_num].turnOff();
}
void RasPiPico::pulseOutput(uint8_t port_num, uint8_t time) {
    this->outputPort[port_num].pulseInProgress = true;
    this->outputPort[port_num].pulseTime = time * 10;
    this->outputPort[port_num].timer = 0;
    dbgserPrint("pulse time: ");
    dbgserPrintln_T(time, HEX);
    toggleOutput(port_num);
}

bool RasPiPico::begin() { // @todo 
    // check to see if Pico is there and responding.

    return true;
}
void RasPiPico::setConfig() { // @todo 
    // set the input / output modes from preferences.
    uint16_t allOutsModes = 0;
    for (uint8_t i = 0; i < 4; i++) {
        allOutsModes |= ((uint16_t) this->pref->outPortModes[i] & 0x000f) << (i * 4);
    }

    Wire.beginTransmission(ADDR_I2C_TO_EXT_BTN_CONTROLLER);
    Wire.write(picoWireCommands::SetOutputMode);
    Wire.write(allOutsModes & 0xff);
    Wire.write((allOutsModes >> 8) & 0xff);
    Wire.endTransmission();
    
    uint16_t allInsModes = 0;
    for (uint8_t i = 0; i < 4; i++) {
        allInsModes |= ((uint16_t) this->pref->extInBtnMode[i] & 0x000f) << (i * 4);
    }

    Wire.beginTransmission(ADDR_I2C_TO_EXT_BTN_CONTROLLER);
    Wire.write(picoWireCommands::SetInputMode);
    Wire.write(allInsModes & 0xff);
    Wire.write((allInsModes >> 8) & 0xff);
    Wire.endTransmission();

    // set update interval
    // this->pref->expPedalUpdateIntervalPref
}
void RasPiPico::update(uint16_t currScn) {
    this->currScn = currScn;
    // dbgserPrintln("RaspiPico update()");
    if (this->updateIntervalTimer > pref->expPedalUpdateIntervalPref) {
        this->updateIntervalTimer = 0;
        // dbgserPrintln("RaspiPico update() - inteval exceeded. proceeding.");
        // get update from PICO, and store values into private members
        int numReadByte;
        byte readBytes[32];
        // uint8_t iterator = 0;
        uint8_t portsToRead = 0;
        uint8_t numberOfBytesToRead = 3;
        for (uint8_t i = 0; i < 4; i++) {
            // if this particular input is set to an expression pedal mode, we need to read 2 more bytes for it. 
            // We get the "button state" values every time, so no need to request them.
            // dbgserPrintln(pref->extInBtnMode[i]);
            if (pref->extInBtnMode[i] == ext_btn_modes::ExpPedalContinuous || pref->extInBtnMode[i] == ext_btn_modes::ExpPedalMinMax) {
                portsToRead |= 1 << i;
                numberOfBytesToRead++; //= 2;
            }
        }

        // Send the request for an update to the PICO.
        Wire.beginTransmission(ADDR_I2C_TO_EXT_BTN_CONTROLLER);
        Wire.write(picoWireCommands::RequestUpdate | portsToRead);
        Wire.endTransmission();

        // Enable i2c timeout and set it to 25ms with a reset if it times out.
        // Wire.setWireTimeout(25000, true);
        Wire.setTimeout(25000);

        // Pico should now allow us to read 1 byte (acknowledge + edge event flag) +
        // 3 bytes(input button states) + 1 bytes for each input configured for an 
        // expression pedal. 
        // Request the bytes and read them into readBytes[].
        numReadByte = Wire.requestFrom(ADDR_I2C_TO_EXT_BTN_CONTROLLER, (int) numberOfBytesToRead);
        for (uint8_t i = 0; i < numReadByte; i++) {
            readBytes[i] = Wire.read();
        }
        // Serial.printf("numReadBytes: %d  numberOfBytesToRead: %d\n", numReadByte, numberOfBytesToRead);
        // check to see if timeout occurred.
        if (numReadByte != numberOfBytesToRead) {
            Serial.printf("i2c bus timed out while reading from PICO.\n");
        } else {
            // if there was no timeout, we should have read the expected number of bytes into readBytes[].

            // first check the edge event flag.
            if ((readBytes[0] & 0x10) == 0x10) {
                dbgserPrint("readBytes[0] (BIN): ");
                dbgserPrintln_T(readBytes[0], BIN);
                dbgserPrint("readBytes[1] (BIN): ");
                dbgserPrintln_T(readBytes[1], BIN);
                dbgserPrint("readBytes[2] (BIN): ");
                dbgserPrintln_T(readBytes[2], BIN);
                // ...then get the data for the ports. 
                uint8_t i = 0;
                for (; i < 4; i++) {
                    if ((readBytes[2] & (1 << i)) > 0) {
                        bool s_tip = (readBytes[1] & (1 << i)) > 0; // get state of tip
                        // bool s_ring = (readBytes[1] & (1 << (i + 4))) > 0; // get state of ring

                        // check to ensure that this port is actually enabled and
                        // invert s_tip if polarity inversion is enabled (s_tip != this->pref->extInBtnPolInv[i], has the effect of inverting s_tip when polinv is true.)
                        // then send the btn number / identifier and the current scene number to the handler function.
                        if ((s_tip != this->pref->extInBtnPolInv[i]) &&
                            ((this->pref->extInBtnMode[i] == ext_btn_modes::DualButton) || (this->pref->extInBtnMode[i] == ext_btn_modes::SingleButton))
                            ) {
                            dbgserPrint("ext btn mode: ");
                            dbgserPrintln_T(this->pref->extInBtnMode[i], HEX);
                            this->qhandler_func(i, this->currScn, 0);
                        }
                        // if ((s_ring != this->pref->extInBtnPolInv[i]) && (this->pref->extInBtnMode[i] == ext_btn_modes::DualButton)) {
                        //     this->qhandler_func(i + 4, this->currScn, 0);
                        // }
                    }
                }
                for (; i < 8; i++) {
                    if ((readBytes[2] & (1 << i)) > 0) {
                        // bool s_tip = (readBytes[1] & (1 << i)) > 0; // get state of tip
                        bool s_ring = (readBytes[1] & (1 << i)) > 0; // get state of ring

                        // check to ensure that this port is actually enabled and
                        // invert s_tip if polarity inversion is enabled (s_tip != this->pref->extInBtnPolInv[i], has the effect of inverting s_tip when polinv is true.)
                        // then send the btn number / identifier and the current scene number to the handler function.
                        // if ((s_tip != this->pref->extInBtnPolInv[i]) && ((this->pref->extInBtnMode[i] == ext_btn_modes::DualButton) || (this->pref->extInBtnMode[i] == ext_btn_modes::SingleButton))) {
                        //     this->qhandler_func(i, this->currScn, 0);

                        // }
                        if ((s_ring != this->pref->extInBtnPolInv[i - 4]) && (this->pref->extInBtnMode[i - 4] == ext_btn_modes::DualButton)) {
                            dbgserPrint("ext btn mode: ");
                            dbgserPrintln_T(this->pref->extInBtnMode[i - 4], HEX);
                            this->qhandler_func(i, this->currScn, 0);
                        }
                    }
                }
            }

            // Parse expression pedal data. 
            // @todo 
            uint8_t indexCount = 3;
            for (uint8_t i = 0; i < 4; i++) {
                // bits 0:4 of readByte[0] are a mask the port numbers that were requested. 
                if ((readBytes[0] & (1 << i)) > 0) {
                    // readByte[1] and readByte[2] have data for button input types of all 4 ports. 
                    // Therefore we start at byte 4
                    if (this->extIns[i].getExpVal() != readBytes[indexCount]) {
                        this->extIns[i].setExpVal(readBytes[indexCount]);
                        this->qhandler_func(i + 8, this->currScn, this->extIns[i].getExpVal());
                    }
                    indexCount++;
                }
            }
        }
    }

    // check to see if any pulses are in progress, and if so, check to see if it has expired.
    for (uint8_t i = 0; i < 8; i++) {
        if (this->outputPort[i].pulseInProgress && (this->outputPort[i].timer > this->outputPort[i].pulseTime)) {
            toggleOutput(i);
            this->outputPort[i].pulseInProgress = false;
        }
    }

    // check to see if any state changes are pending.
    // have to use separate BOOL because pending does not necessarily indicate that a change is pending. It will indicate the requested states.
    bool isAnyPending = false;
    uint8_t pending = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (this->outputPort[i].changePending) {
            this->outputPort[i].changePending = false;
            isAnyPending = true;
            dbgserPrint("pending: ");
            dbgserPrintln_T(pending, BIN);
        }
        pending |= this->outputPort[i].OR_state();
    }
    if (isAnyPending) {
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

void ExtPedalInput::setState(bool tip, bool ring) {
    this->state_R.state_CLOSED = ring;
    this->state_R.state_OPEN = !ring;
    this->state_T.state_CLOSED = tip;
    this->state_T.state_OPEN = !tip;
}
bool ExtPedalInput::tipState() {
    return this->state_T.state_CLOSED;
}
bool ExtPedalInput::ringState() {
    return this->state_R.state_CLOSED;
}
bool ExtPedalInput::isTipClosed() {
    return this->state_T.state_CLOSED;
}
bool ExtPedalInput::isTipOpen() {
    return this->state_T.state_OPEN;
}
bool ExtPedalInput::isRingClosed() {
    return this->state_R.state_CLOSED;
}
bool ExtPedalInput::isRingOpen() {
    return this->state_R.state_OPEN;
}

void ExtPedalInput::setExpVal(uint8_t val) {
    this->expressionVal = val;
}

uint8_t ExtPedalInput::getExpVal() {
    return this->expressionVal;
}


void buttonActions::resetToDefaults() {
    this->action = ActionTypes::NULL_Action;
    for (int i = 0; i < 16; i++) {
        this->actionData[i] = 0;
    }
}
///\brief Do this action's thing. Send some data, toggle a thing, etc.
bool buttonActions::doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevice, qdAction& actQ, void (*SceneChange_F)(int), PrefsObj& prefs_R, RasPiPico& pico_R, uint16_t currScn) {
    midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> devArr[] = { midiDevice };
    return doAction(devArr, actQ, SceneChange_F, prefs_R, pico_R, currScn, 0, true);
}
///\brief Do this action's thing. Send some data, toggle a thing, etc.
bool buttonActions::doAction(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[], qdAction& actQ, void (*SceneChange_F)(int), PrefsObj& prefs_R, RasPiPico& pico_R, uint16_t currScn, uint8_t value, bool singleMidi) {
    actQ.hasBeenSent = true;
    dbgserPrintln("do Action");
    dbgserPrint("this->action: ");
    dbgserPrintln(this->action);
    actQ.elapsedSinceActionCalled = 0; // reset for every action.
    switch (this->action) {
    case ActionTypes::NULL_Action: {
        return false;
        break;
    }
    case ActionTypes::SendNoteOn: {
        byte midiOutPort = this->actionData[0];
        byte midiChan = this->actionData[1];
        byte noteVal = this->actionData[2];
        byte velocity = this->actionData[3];
        midiDevs[midiOutPort].sendNoteOn(noteVal, velocity, midiChan);
        break;
    }
    case ActionTypes::SendNoteOff: {
        byte midiOutPort = this->actionData[0];
        byte midiChan = this->actionData[1];
        byte noteVal = this->actionData[2];
        byte velocity = this->actionData[3];
        midiDevs[midiOutPort].sendNoteOff(noteVal, velocity, midiChan);
        break;
    }
    case ActionTypes::SendCC_AbsoluteValue: {
        byte midiOutPort = this->actionData[0];
        byte midiChan = this->actionData[1];
        byte midiCC = this->actionData[2];
        byte midiCCval = this->actionData[3];
        if (midiCCval == 128) midiCCval = value;
        midiDevs[midiOutPort].sendControlChange(midiCC, midiCCval, midiChan);
        break;
    }
    case ActionTypes::SendCC_Increment0_127... ActionTypes::SendCC_Decrement96_97: {
        byte midiOutPort = this->actionData[0];
        byte midiChan = this->actionData[1];
        byte midiCC = this->actionData[2];
        const byte lut[] = { 127, 0, 64, 63, 97, 96 };
        byte midiCCval = lut[this->action - ActionTypes::SendCC_Increment0_127];
        midiDevs[midiOutPort].sendControlChange(midiCC, midiCCval, midiChan);
        break;
    }
    case ActionTypes::SendProgChange: {
        byte midiOutPort = this->actionData[0];
        byte midiProgNum = this->actionData[1];
        byte midiChan = this->actionData[2];
        midiDevs[midiOutPort].sendProgramChange(midiProgNum, midiChan);
        break;
    }
    case ActionTypes::NextScene: {
        // setCurrentScene(currScn+1);
        SceneChange_F(currScn + 1);
        break;
    }
    case ActionTypes::PrevScene: {
        // setCurrentScene(currScn-1);
        SceneChange_F(currScn - 1);
        break;
    }
    case ActionTypes::TurnOnOutPort: {
        // pico_R.setOutputOn((this->actionData[0] * 2) + this->actionData[1]);
        pico_R.setOutputOn((this->actionData[0]) + this->actionData[1] * 4);
        break;
    }
    case ActionTypes::TurnOffOutPort: {
        // pico_R.setOutputOff((this->actionData[0] * 2) + this->actionData[1]);
        pico_R.setOutputOff((this->actionData[0]) + this->actionData[1] * 4);
        break;
    }
    case ActionTypes::ToggleOutPort: {
        // pico_R.toggleOutput((this->actionData[0] * 2) + this->actionData[1]);
        pico_R.toggleOutput((this->actionData[0]) + this->actionData[1] * 4);
        break;
    }
    case ActionTypes::PulseOutPort: {
        pico_R.pulseOutput(this->actionData[0] + (this->actionData[1] * 4), this->actionData[2]);
        break;
    }
    case ActionTypes::ActionWait: {
        unsigned long minutes = this->actionData[0];
        unsigned long seconds = this->actionData[1];
        unsigned long milliseconds = this->actionData[2] * 10;
        actQ.timeToWait = milliseconds + (seconds * 1000) + (minutes * 60 * 1000);
        dbgserPrint("actQ.timetoWait: ");
        dbgserPrintln_T(actQ.timeToWait, HEX);
        break;
    }
    case ActionTypes::JumpToScene: {
        uint16_t scnNum = 0;
        if (this->actionData[1] == 0) {
            scnNum = this->actionData[0];
        } else {
            scnNum = this->actionData[0] << 8;
            scnNum += this->actionData[1];
        }
        // SceneChange_F(scnNum);
        break;
    }
    case ActionTypes::ExpPedalUpdateInterval: {
        prefs_R.expPedalUpdateIntervalPref = this->actionData[0];
        break;
    }
    case ActionTypes::SendPlay: {
        break;
    }
    case ActionTypes::SendStop: {
        break;
    }
    case ActionTypes::SendContinue: {
        break;
    }
    case ActionTypes::SeekToBeginning: {
        break;
    }
    case ActionTypes::SendCustomMidiMessage: {
        break;
    }
    case ActionTypes::MidiMangler_Start_Something: {
        break;
    }
    case ActionTypes::MidiMangler_Stop_Something: {
        break;
    }
    case ActionTypes::MidiMangler_OneShot: {
        break;
    }
    default:
        Serial.printf("Action value not recognized.");
        return false;
        break;
    }
    return true;
}

void MAIN_BTN::resetToDefaults() {
    char defaultText[] = "Button disabled.";
    for (int i = 0; i < 17; i++) {
        this->topRowText[i] = defaultText[i];
    }
    for (int i = 0; i < 32; i++) {
        this->Actions[i].resetToDefaults();
    }
}

void EXT_BTN::resetToDefaults() {
    this->Btn_Mode = ext_btn_modes::Disabled;
    for (int i = 0; i < 32; i++) {
        this->Actions[i].resetToDefaults();
    }
}

void OUT_PORTS::resetToDefaults() {
    this->out_mode = out_port_modes::Disable;
    this->state = out_port_state::OutPortStateNotSaved;
}

void SCENE::resetToDefaults() {
    for (int i = 0; i < 10; i++) {
        this->mainButtons[i].resetToDefaults();
    }
    for (int i = 0; i < 4; i++) {
        this->output_ports[i].resetToDefaults();
    }
    for (int i = 0; i < 8; i++) {
        this->extButtons[i].resetToDefaults();
    }
}
void SceneObjSingle::resetToDefaults() {
    for (int i = 0; i < 10; i++) {
        this->mainButtons[i].resetToDefaults();
    }
    for (int i = 0; i < 4; i++) {
        this->output_ports[i].resetToDefaults();
    }
    for (int i = 0; i < 8; i++) {
        this->extButtons[i].resetToDefaults();
    }
}
void SceneObjSingle::initValues() {
    for (int u = 0; u < 8; u++) {
        for (int p = 0; p < 32; p++) {
            this->extButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
            for (int q = 0; q < 16; q++) {
                this->extButtons[u].Actions[p].actionData[q] = 0;
            }
        }
        this->extButtons[u].Btn_Mode = ext_btn_modes::Disabled;
    }
    for (int u = 0; u < 4; u++) {
        this->output_ports[u].out_mode = out_port_modes::DualOutput;
        this->output_ports[u].state = out_port_state::Off;
    }
    for (int u = 0; u < 10; u++) {
        for (int p = 0; p < 32; p++) {
            this->mainButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
            for (int q = 0; q < 16; q++) {
                this->mainButtons[u].Actions[p].actionData[q] = 0;
            }
        }
        for (int p = 0; p < 50; p++) {
            this->mainButtons[u].topRowText[p] = '\0';
        }
    }
};

SceneObj::SceneObj() {
    this->numScenes = MAX_NUMBER_OF_SCENES;
}
SceneObj::SceneObj(int numberOfScenes) {
    this->numScenes = numberOfScenes;
}

void SceneObj::initValues() {
    for (int i = 0; i < this->numScenes; i++) {
        for (int u = 0; u < 8; u++) {
            for (int p = 0; p < 32; p++) {
                this->scenesArr[i].extButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                for (int q = 0; q < 16; q++) {
                    this->scenesArr[i].extButtons[u].Actions[p].actionData[q] = 0;
                }
            }
            this->scenesArr[i].extButtons[u].Btn_Mode = ext_btn_modes::Disabled;
        }
        for (int u = 0; u < 4; u++) {
            this->scenesArr[i].output_ports[u].out_mode = out_port_modes::DualOutput;
            this->scenesArr[i].output_ports[u].state = out_port_state::Off;
        }
        for (int u = 0; u < 10; u++) {
            for (int p = 0; p < 32; p++) {
                this->scenesArr[i].mainButtons[u].Actions[p].action = buttonActions::ActionTypes::NULL_Action;
                for (int q = 0; q < 16; q++) {
                    this->scenesArr[i].mainButtons[u].Actions[p].actionData[q] = 0;
                }
            }
            for (int p = 0; p < 50; p++) {
                this->scenesArr[i].mainButtons[u].topRowText[p] = '\0';
            }
        }
    }
}

buttonActionQueue::buttonActionQueue(bool isMain, SceneObj& scenes, midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> midiDevs[], void (*SceneChange_F)(int), PrefsObj& pref_R) {
    this->isMainButtonQueue = isMain;
    this->pref_R = &pref_R;
    this->scenes = &scenes;
    this->SceneChange_F = SceneChange_F;
    this->midiDevs = midiDevs;
}

void buttonActionQueue::setPicoObj(RasPiPico& pico_R) {
    this->pico_obj_P = &pico_R;
}

int buttonActionQueue::addAction(uint8_t btn, uint8_t num, uint16_t scene, uint8_t val = 0) {
    if (this->lastIn == this->firstOut - 1)
        return 0; // queue is full
    this->actionsQ[this->lastIn].actionNum = num;
    this->actionsQ[this->lastIn].buttonIdNum = btn;
    this->actionsQ[this->lastIn].scnNum = scene;
    this->actionsQ[this->lastIn].hasBeenSent = false;
    this->actionsQ[this->lastIn].value = val;
    this->lastIn++;
    return this->lastIn - this->firstOut;
}

int buttonActionQueue::processQueue(uint16_t currScn) {
    if (this->firstOut == this->lastIn) {
        //queue is empty
        return 0;
    }
    uint8_t btn = this->actionsQ[this->firstOut].buttonIdNum;
    uint8_t act = this->actionsQ[this->firstOut].actionNum;
    uint16_t scn = this->actionsQ[this->firstOut].scnNum;
    uint8_t val = this->actionsQ[this->firstOut].value;

    // check to see if this act is a "wait action"
    // if so, check elapsed time since relevent action (first or previous)
    bool isWait;
    if (this->isMainButtonQueue) {

        isWait = (buttonActions::ActionTypes::ActionWait == scenes->scenesArr[scn].mainButtons[btn].Actions[act].action);
    } else {
        isWait = (buttonActions::ActionTypes::ActionWait == scenes->scenesArr[scn].extButtons[btn].Actions[act].action);
    }
    if (isWait) {
        // actionData[15] shall hold a value to determine if the wait has started.
        bool isSent = this->actionsQ[this->firstOut].hasBeenSent;
        // if this is first time we've been here, call doAction() for the wait action to set the time it was called.
        if (!isSent) {
            if (this->isMainButtonQueue) {
                // dbgserPrintln("isFirst was true.");
                scenes->scenesArr[scn].mainButtons[btn].Actions[act].doAction(midiDevs, this->actionsQ[this->firstOut], SceneChange_F, *pref_R, *pico_obj_P, currScn, val);
            } else {
                scenes->scenesArr[scn].extButtons[btn].Actions[act].doAction(midiDevs, this->actionsQ[this->firstOut], SceneChange_F, *pref_R, *pico_obj_P, currScn, val);
            }
        }
        if (this->actionsQ[this->firstOut].elapsedSinceActionCalled > (this->actionsQ[this->firstOut].timeToWait * 0.95)) {
            this->firstOut++;
        }
        return this->lastIn - this->firstOut;
    }

    // if it was not a wait action, call doAction for this action.
    if (this->isMainButtonQueue) {
        scenes->scenesArr[scn].mainButtons[btn].Actions[act].doAction(midiDevs, this->actionsQ[this->firstOut], SceneChange_F, *pref_R, *pico_obj_P, currScn, val);
    } else {
        scenes->scenesArr[scn].extButtons[btn].Actions[act].doAction(midiDevs, this->actionsQ[this->firstOut], SceneChange_F, *pref_R, *pico_obj_P, currScn, val);
    }
    this->firstOut++;
    return this->lastIn - this->firstOut;
}
