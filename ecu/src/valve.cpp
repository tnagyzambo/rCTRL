#include<valve.hpp>

using namespace machinecontrol;


// Templated based on the valve type and the output pin
// the state being true means it is open
// The valve type can be NO or NC
/* A template for the class valve. It is a template because it is a class that can be used for multiple
types of valves. The template is based on the valve type and the output pin. */


bool valveInterface::isPowered() const {
    return power;
}
bool valveInterface::isOpen() const {
    return state;
}

void NormOpen::open() {
    digital_outputs.set(PIN,LOW);
    state = true;
}

void NormOpen::close() {
    digital_outputs.set(PIN,HIGH);
    state = false;
}

void NormClosed::open() {
    digital_outputs.set(PIN,HIGH);
    state = true;
}

void NormClosed::close() {
    digital_outputs.set(PIN,LOW);
    state = false;
}

sequenceData::sequenceData(long int actuationTime, std::shared_ptr<valveInterface> valveID, valveAction action){
        actuationTime = actuationTime;
        valveID = valveID;
        action = action;
}
sequenceData::~sequenceData(){
    valveID.reset();
}

void autoSequence::addEvent(long int time, std::shared_ptr<valveInterface> valveID, valveAction action) {
    SequenceVector.push_back(sequenceData(time, valveID, action));
}

int autoSequence::runSequence(long int currentTime, int dataCase) {
    if (SequenceVector.empty()) {
        //Return to 0 state
        //Stop sequence
        return 0;
    }
    //TODO add start state check.
    if (!sequenceActive) {
        sequenceActive = true;
        sequenceStartTime = currentTime;
    }

    for (auto eventPtr = std::begin(SequenceVector); eventPtr != std::end(SequenceVector); ++eventPtr) {
        
        // Check if the actuation time is good AND that true time is -INFINITY
        if (eventPtr->actuationTime >= (currentTime-sequenceStartTime) && eventPtr->trueTime < 0) {
            // Open or close valve
            if (eventPtr->action == openValve) {
                eventPtr->valveID->open();
            } else if(eventPtr->action == closeValve) {
                eventPtr->valveID->close();
            }
            // Set trueTime so we don't keep calling it
            eventPtr->trueTime = currentTime;
            ++eventCounter;
        }
    }

    // Return dataCase if we continue with sequence
    // Return 0 if we are done
    if (eventCounter >= SequenceVector.size()){
        return 0; // Sequence over
    } else {
        return dataCase; // Continue sequence
    }
}

void autoSequence::resetSequence(){
    // Set all times back to negative infinity
    for (auto eventPtr = std::begin(SequenceVector); eventPtr != std::end(SequenceVector); ++eventPtr) {
        eventPtr->trueTime = -INFINITY;
    }
    sequenceStartTime = INFINITY;
    eventCounter = 0;
}