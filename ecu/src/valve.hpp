#pragma once

#include<Arduino_MachineControl.h>
#include<vector>

enum valveState { Open, Closed};
enum valveAction { openValve, closeValve};

using namespace machinecontrol;


// Templated based on the valve type and the output pin
// the state being true means it is open
// The valve type can be NO or NC
/* A template for the class valve. It is a template because it is a class that can be used for multiple
types of valves. The template is based on the valve type and the output pin. */


class valveInterface {
    protected:
        //Default is unpowered
        bool power, state; //true is open, true is powered

    public:
        const int PIN;
        valveInterface(int PIN):PIN(PIN){};
       /* A function that opens the valve. This is templated based on if the valve is normally open or normally closed*/
        virtual void open() = 0;

        /* A function that closes the valve. This is templated based on if the valve is normally open or normally closed*/
        virtual void close() = 0;

        /**
         * This function returns a boolean value that indicates whether the valve is powered on or not
         * 
         * @return A boolean value.
         */
        bool isPowered() const;
        /**
         * It returns the state of the valve.
         * 
         * @return A boolean value.
         */
        bool isOpen() const;
};

class NormOpen: public valveInterface {
    private:
        //Default state
        bool power = false;
        bool state = true;

    public:
        NormOpen(int PIN):valveInterface(PIN){};
        void open() override final;

        void close() override final;
};


class NormClosed: public valveInterface {
    private:
        //Default state
        bool power = false;
        bool state = false;

    public:
    
        NormClosed(int PIN):valveInterface(PIN){};
        void open() override final;

        void close() override final;
};


struct sequenceData {
    long int actuationTime;
    std::shared_ptr<valveInterface> valveID;
    valveAction action;
    long int trueTime = -INFINITY;

    sequenceData(long int actuationTime, std::shared_ptr<valveInterface> valveID, valveAction action);

    ~sequenceData();
};


class autoSequence {
    // configuration: Store a list of time : valve : action : (Time performed)
    // 
    // Check if valve are in starting positions. Return why its not possible to start true/false : reason
    // Run sequence if commanded and check ok:
    //  Store start time
    //  Check actions against elapsed time
    //  Perform actions when it is okay. (only do them once) Do this writing to the time perfomed
    // 
    // Reset auto sequence

    //using enum valveAction;
    //Create empty sequence variable
    std::vector<sequenceData> SequenceVector;
    long int sequenceStartTime = INFINITY;
    bool sequenceActive = false;
    unsigned int eventCounter;

    public:
        void addEvent(long int time, std::shared_ptr<valveInterface> valveID, valveAction action);

        int runSequence(long int currentTime, int dataCase);
        
        void resetSequence();
};
