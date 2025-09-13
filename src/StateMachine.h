#pragma once

class StateMachine{

public:
    bool endFireFlag;
    bool valveStateChange;
    bool getFire;
    bool breakWireStatus; 
    bool keySwitchStatus;

    // relay states/
    bool isok_state;
    bool isol_state;
    bool maink_state;
    bool mainl_state;
    bool ventk_state;
    bool ventl_state;
    bool purge_state;

    unsigned long referenceTime;


    // origin -> default state
    // pressurize -> armed || depressurize -> default
    // fire -> hot
    // abort -> default
    // full -> manual

    enum State {DEFAULT, ARMED, HOT, MANUAL, KEY};
    enum Command {ORIGIN = 13, PRESSURIZE = 12, FIRE = 14, DEPRESSURIZE = 15, ABORT = 16, FULL = 11};    

    StateMachine(const int keroIso, const int loxIso, const int keroMain, const int loxMain,
                            const int keroVent, const int loxVent, const int purge, const int keySwitch,
                            const int breakWire, const int igniter);

    void initializeMachina();
    void processCommand(int command, unsigned long targetTime, unsigned long purgeTime, unsigned long elapsedTime,
                        unsigned long keroDelay); //what to do when receive command
    void changeState(State newState); // given state and command, change to appropriate state STATUS
    int getState(); // returns the current state
    bool getBreakWire();
    bool getKeySwitch();

    void reset();
    void pressurize();
    void depressurize();
    void abort();
    void endFire();
    void purge();

private:
    /*======== Relays ========*/
    int _keroIsolation; // isok
    int _loxIsolation; // isol
    int _keroMain; // main k
    int _loxMain; // main l
    int _keroVent; // vent k
    int _loxVent; // vent l
    int _purge; //purge  

    int _keySwitch; //safety key switch  
    int _igniter;
    int _breakWire;

    State _state;
};
