#include "StateMachine.h"
#include "Arduino.h" 

StateMachine::StateMachine(const int keroIso, const int loxIso, const int keroMain, const int loxMain,
                            const int keroVent, const int loxVent, const int purge, const int keySwitch,
                            const int breakWire ,const int igniter)
{
    _keroIsolation = keroIso;
    _loxIsolation = loxIso;
    _keroMain = keroMain;
    _loxMain = loxMain;
    _keroVent = keroVent;
    _loxVent = loxVent;
    _purge = purge;
    _keySwitch = keySwitch;
    _igniter = igniter;
    _breakWire = breakWire;
}

void StateMachine::initializeMachina()
{
    pinMode(_keroIsolation,OUTPUT);
    pinMode(_loxIsolation,OUTPUT);
    pinMode(_keroMain,OUTPUT);
    pinMode(_loxMain,OUTPUT);
    pinMode(_keroVent,OUTPUT);
    pinMode(_loxVent,OUTPUT);
    pinMode(_purge,OUTPUT);   
    pinMode(_igniter, OUTPUT);
    pinMode(_keySwitch, INPUT_PULLDOWN);
    pinMode(_breakWire, INPUT_PULLDOWN);

    isok_state = 0;
    isol_state = 0;
    maink_state = 0;
    mainl_state = 0;
    ventk_state = 0;
    ventl_state = 0;
    purge_state = 0;
    endFireFlag = 0;
    valveStateChange = 1;
    referenceTime = 0;
    getFire = 0;
    breakWireStatus = digitalRead(_breakWire);
    keySwitchStatus = digitalRead(_keySwitch);

    _state = DEFAULT;
}

void StateMachine::changeState(State newState)
{
    _state = newState;
}

void StateMachine::reset()
{
    digitalWrite(_keroIsolation, LOW);
    digitalWrite(_loxIsolation, LOW);
    digitalWrite(_keroMain, LOW);
    digitalWrite(_loxMain, LOW);
    digitalWrite(_keroVent, LOW);
    digitalWrite(_loxVent, LOW);
    digitalWrite(_purge, LOW);

    isok_state = 0;
    isol_state = 0;
    maink_state = 0;
    mainl_state = 0;
    ventk_state = 0;
    ventl_state = 0;
    purge_state = 0;
}

void StateMachine::pressurize()
{
    digitalWrite(_keroIsolation, HIGH);
    digitalWrite(_loxIsolation, HIGH);
    digitalWrite(_keroVent, HIGH);
    digitalWrite(_loxVent, HIGH);

    isok_state = 1;
    isol_state = 1;
    ventk_state = 1;
    ventl_state = 1;
}

void StateMachine::depressurize()
{
    digitalWrite(_keroIsolation, LOW);
    digitalWrite(_loxIsolation, LOW);
    digitalWrite(_keroVent, LOW);
    digitalWrite(_loxVent, LOW);

    isok_state = 0;
    isol_state = 0;
    ventk_state = 0;
    ventl_state = 0;
}

void StateMachine::endFire()
{
    digitalWrite(_keroMain, LOW);
    digitalWrite(_loxMain, LOW);

    maink_state = 0;
    mainl_state = 0;
}

void StateMachine::purge()
{
    digitalWrite(_purge, HIGH);

    purge_state = 1;
}

void StateMachine::abort()
{
    digitalWrite(_keroMain, LOW);
    digitalWrite(_loxMain, LOW);
    digitalWrite(_keroIsolation, LOW);
    digitalWrite(_loxIsolation, LOW);
    digitalWrite(_keroVent, LOW);
    digitalWrite(_loxVent, LOW);
    digitalWrite(_purge, HIGH);

    isok_state = 0;
    isol_state = 0;
    maink_state = 0;
    mainl_state = 0;
    ventk_state = 0;
    ventl_state = 0;
    purge_state = 1;
}

void StateMachine::processCommand(int command, unsigned long targetTime, unsigned long purgeTime, 
                                unsigned long timeElapsed, unsigned long keroDelay)
{
    switch(_state)
    {
        case DEFAULT:
            if(!digitalRead(_keySwitch))
            {
                changeState(KEY);
                abort();
                keySwitchStatus = 0;
                valveStateChange = 1;
            }
            if(command == PRESSURIZE)
            {
                changeState(ARMED);  
                pressurize();
                valveStateChange = 1; 
            }
            if (command == FULL)
            {
                changeState(MANUAL);
                valveStateChange = 1; 
            }
            if(purge_state == 1 && timeElapsed >= 4000)
            {
                digitalWrite(_purge, LOW);
                endFireFlag = 0;
                purge_state = 0;
                valveStateChange = 1;
            }
            break;

        case ARMED:
            if(!digitalRead(_keySwitch))
            {
                changeState(KEY);
                abort();
                keySwitchStatus = 0;
                valveStateChange = 1;
            }
            if(command == FIRE)
            {
                digitalWrite(_igniter, HIGH);
                referenceTime = millis();
                changeState(HOT);
                valveStateChange = 1; 
            }
            if(command == DEPRESSURIZE)
            {
                changeState(DEFAULT);
                depressurize();
                valveStateChange = 1;
            }
            if (command == FULL)
            {
                changeState(MANUAL);
                valveStateChange = 1;
            }
            break;

        case HOT:
            if(!digitalRead(_keySwitch))
            {
                changeState(KEY);
                abort();
                keySwitchStatus = 0;
                valveStateChange = 1;
            }
            if(command == ABORT)
            {
                abort();
                changeState(DEFAULT);
                referenceTime = millis();
                endFireFlag = 1;
                valveStateChange = 1;
            }
            if (command == FULL)
            {
                changeState(MANUAL);
                valveStateChange = 1; 
            }
            if(timeElapsed <= 3000 && !digitalRead(_breakWire) && mainl_state == 0 && !endFireFlag)
            {
                digitalWrite(_loxMain, HIGH);
                digitalWrite(_igniter, LOW);
                mainl_state = 1;  
                getFire = 1;
                valveStateChange = 1; 
                referenceTime = millis();
            }
            else if(timeElapsed >= keroDelay && mainl_state == 1 && maink_state == 0 && getFire && !endFireFlag)
            {
                digitalWrite(_keroMain, HIGH);
                maink_state = 1;
                valveStateChange = 1;
            }
            else if (timeElapsed >= targetTime && !endFireFlag && getFire)
            {
                endFire();
                purge();
                referenceTime = millis();
                endFireFlag = 1;
                getFire = 0;
                valveStateChange = 1;
            }
            else if (timeElapsed > purgeTime && endFireFlag && !getFire)
            {
                reset();
                changeState(DEFAULT);
                endFireFlag = 0;
                valveStateChange = 1;
            }
            else if(timeElapsed > 3000 && digitalRead(_breakWire) && mainl_state ==0)
            {
                changeState(ARMED);
                digitalWrite(_igniter, LOW);
                endFireFlag = 0;
                valveStateChange = 1;
            }          
            break;

        case MANUAL:
            if(!digitalRead(_keySwitch))
            {
                changeState(KEY);
                abort();
                keySwitchStatus = 0;
                valveStateChange = 1;
            }
            if(command == ORIGIN)
            {
                reset();
                valveStateChange = 1;
                changeState(DEFAULT);
            }
            if(command == PRESSURIZE && isok_state && isol_state && !maink_state && !mainl_state)
            {
                changeState(ARMED);
                valveStateChange = 1;
            }
            if(command == 1) // relay 1 on
            {
                isok_state = !isok_state;
                digitalWrite(_keroIsolation, isok_state);
                valveStateChange = 1;
            }
            else if(command == 2) // relay 2 on
            {
                isol_state = !isol_state;
                digitalWrite(_loxIsolation, isol_state);
                valveStateChange = 1;
            }
            else if(command == 3) // relay 3 on
            {
                maink_state = !maink_state;
                digitalWrite(_keroMain, maink_state);
                valveStateChange = 1;
            }  
            else if(command == 4) // relay 4 on
            {
                mainl_state = !mainl_state;
                digitalWrite(_loxMain, mainl_state);
                valveStateChange = 1;
            }  
            else if(command == 5) // relay 5 on
            {
                ventk_state = !ventk_state;
                digitalWrite(_keroVent, ventk_state);
                valveStateChange = 1;
            }
            else if(command == 6) // relay 6 on
            {
                ventl_state = !ventl_state;
                digitalWrite(_loxVent, ventl_state);
                valveStateChange = 1;
            }
            else if(command == 7) 
            {
                purge_state = !purge_state;
                digitalWrite(_purge, purge_state);
                valveStateChange = 1;
            }
            break;
        case KEY:
            if(digitalRead(_keySwitch))
            {
                keySwitchStatus = 1;
                changeState(DEFAULT);
                valveStateChange = 1;
            }
            break;
    }
}

bool StateMachine::getBreakWire()
{
    breakWireStatus = digitalRead(_breakWire);

    return breakWireStatus;
}

bool StateMachine::getKeySwitch()
{
    keySwitchStatus = digitalRead(_keySwitch);

    return keySwitchStatus;
}

int StateMachine::getState()
{
    // default = 0
    // armed = 1
    // hot = 2
    // manual = 3
    // key = 4
    return _state;
}

    