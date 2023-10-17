
#include "abs_stepper.h"

// limit (HARD) switch setup 
void AbsStepper::setLimitSwPin(short pin_, long lim_pos, LimitSWPinSelect pin_select, short active_state, LimitSWMethod method){
    // Serial.println("***lim_abs_step lim up: " + String(lim_abs_step));
    if(pin_select == LIMIT_DOWN_PIN){
        _limitsw_down_pin = pin_;
        limitsw_active_state = active_state;
    } else{
        _limitsw_up_pin = pin_;
        limitsw_active_state = active_state;
    }
    // Serial.println("***setuppin step: " + String(lim_abs_step) + "  pin_select" + String(pin_select));
    setHardLimAsPos(lim_pos, pin_select);
    pinMode(pin_, INPUT_PULLUP);

    // if(method == POLL){

    // }
}

// Overload: use joint deg (double) instead of abs steps
void AbsStepper::setLimitSwPin(short pin_, double lim_jdeg, LimitSWPinSelect pin_select, short active_state, LimitSWMethod method){
    // Serial.println("***JDEG lim up: " + String((float)jdeg));
    // setLimitSwPin(pin_, (long)(jdeg * stepper2joint_ratio * _step_per_deg), pin_select, active_state, method);
    setLimitSwPin(pin_, convertJdeg2Pos(lim_jdeg), pin_select, active_state, method);

}

void AbsStepper::setHardLimUnivPin(short pin_, long min_pos, long max_pos, short active_state, LimitSWMethod method){
    univ_hard_lim = LIMIT_PIN_MAX_AS_UNIV;
    if (min_pos < max_pos){
        setLimitSwPin(pin_, min_pos, LIMIT_DOWN_PIN, active_state, method);
        setLimitSwPin(pin_, max_pos, LIMIT_UP_PIN, active_state, method);
    } else if (min_pos > max_pos){
        setLimitSwPin(pin_, max_pos, LIMIT_DOWN_PIN, active_state, method);
        setLimitSwPin(pin_, min_pos, LIMIT_UP_PIN, active_state, method);
    } else{
        Serial.println("***** ERROR: Min and Max hard limits CANNOT be equal!");
    }
    
}

void AbsStepper::setHardLimUnivPin(short pin_, double min_jdeg, double max_jdeg, short active_state, LimitSWMethod method){
    setHardLimUnivPin(pin_, convertJdeg2Pos(min_jdeg), convertJdeg2Pos(max_jdeg), active_state, method);
}

void AbsStepper::setHardLimAsPos(long lim_pos, LimitSWPinSelect pin_select){
    if(pin_select == LIMIT_DOWN_PIN){
        if(_limitsw_up_pin != SHRT_MIN){                     // there is a uppper limit switch value, so check it first before setting lower limit
            if(lim_pos < _lim_max_hard_pos){            // check if lower limit is lower than upper limit
                _lim_min_hard_pos = lim_pos;
                // Serial.println("(lim_down < lim_up OK) Set lim down: " + String(_lim_max_hard_pos));
            } else {
                // error - lower limit is higher than upper limit
                Serial.println("***** ERROR: Limit switch reset position error.");
            }
        } else{                                             // no upper limit, set up lower limit immediately
            _lim_min_hard_pos = lim_pos;
            // Serial.println("Set lim down DIRECTLY:" + String(_lim_min_hard_pos));
        }
        
    } else{
        if(_limitsw_down_pin != SHRT_MIN){                   // there is a lower limit switch value, so check it first before setting upper limit
            if(lim_pos > _lim_min_hard_pos){          // check if upper limit is higher than lower limit
                // Serial.println("Setting lim up to: " + String(a_step));
                _lim_max_hard_pos = lim_pos;
                // Serial.println("(lim_down < lim_up OK) Set lim up: " + String(_lim_max_hard_pos));
            } else {
                // error - upper limit is lower than lower limit
                Serial.println("***** ERROR: Limit switch reset position error.");
            }
        } else{                                             // no lower limit, set up upper limit immediately
            // Serial.println("Setting lim up to: %l" + String(a_step));
            _lim_max_hard_pos = lim_pos;
            // Serial.println("Set lim up DIRECTLY:" + String(_lim_max_hard_pos));
        }
    }
}

void AbsStepper::setHardLimUpPin(short pin_, long lim_pos, short active_state, LimitSWMethod method){
    // Serial.println("abs lim up: " + String(lim_abs_step));
    setLimitSwPin(pin_, lim_pos, LIMIT_UP_PIN, active_state, method);
}
void AbsStepper::setHardLimUpPin(short pin_, double lim_jdeg, short active_state, LimitSWMethod method){
    // Serial.println("jdeg lim up: " + String(jdeg));
    setLimitSwPin(pin_, lim_jdeg, LIMIT_UP_PIN, active_state, method);
}

void AbsStepper::setHardLimUpAsPos(long lim_pos){
    setHardLimAsPos(lim_pos, LIMIT_UP_PIN);
}
void AbsStepper::setHardLimUpAsAdeg(double lim_adeg){
    // setHardLimAsPos((long)(ddeg * _step_per_deg), LIMIT_UP_PIN);
    setHardLimAsPos(convertAdeg2Pos(lim_adeg), LIMIT_UP_PIN);
}
void AbsStepper::setHardLimUpAsJdeg(double lim_jdeg){
    // setHardLimAsPos((long)(jdeg * stepper2joint_ratio * _step_per_deg), LIMIT_UP_PIN);
    setHardLimAsPos(convertJdeg2Pos(lim_jdeg), LIMIT_UP_PIN);
}



void AbsStepper::setHardLimDownPin(short pin_, long lim_pos, short active_state, LimitSWMethod method){
    setLimitSwPin(pin_, lim_pos, LIMIT_DOWN_PIN, active_state, method);
}
void AbsStepper::setHardLimDownPin(short pin_, double lim_jdeg, short active_state, LimitSWMethod method){
    setLimitSwPin(pin_, lim_jdeg, LIMIT_DOWN_PIN, active_state, method);
}

void AbsStepper::setHardLimDownAsPos(long lim_pos){
    setHardLimAsPos(lim_pos, LIMIT_DOWN_PIN);
}
void AbsStepper::setHardLimSWDownAsAdeg(double lim_adeg){
    // setHardLimAsPos((long)(ddeg * _step_per_deg), LIMIT_DOWN_PIN);
    setHardLimAsPos(convertAdeg2Pos(lim_adeg), LIMIT_DOWN_PIN);
}
void AbsStepper::setHardLimDownAsJdeg(double lim_jdeg){
    // setHardLimAsPos((long)(jdeg * stepper2joint_ratio * _step_per_deg), LIMIT_DOWN_PIN);
    setHardLimAsPos(convertJdeg2Pos(lim_jdeg), LIMIT_DOWN_PIN);
}


// long AbsStepper::limitUnivActivatedCB1(){
//     // when limit sw is triggered, STOP ONLY when you are APPROACHING the set position of the limit sw
//     if(steps_remaining > 0 && getDirection() == 1){
//         long retval=stop();
//         // Set current position to set value
//         Serial.println("lim up: " + String(_lim_max_hard_pos));
//         setCurPosAtPos(_lim_max_hard_pos);
//         Serial.println("stop up: " + String(_abs_pos));
//         return retval;
//     } else{
//         return steps_remaining;
//     }   
// }

long AbsStepper::limitUpActivatedCB1(){
    // when limit sw is triggered, STOP ONLY when you are APPROACHING the set position of the limit sw
    if(steps_remaining > 0 && getDirection() == 1){
        long retval=stop();
        // Set current position to set value
        Serial.println("lim up: " + String(_lim_max_hard_pos));
        setCurPosAtPos(_lim_max_hard_pos);
        Serial.println("stop up: " + String(_abs_pos));
        _prev_hlim_triggered = LIMIT_UP_PIN;
        return retval;
    } else{
        return steps_remaining;
    }
    
}

long AbsStepper::limitDownActivatedCB1(){
    if(steps_remaining > 0 && getDirection() == -1){
        long retval=stop();
        // Set current position to set value
        setCurPosAtPos(_lim_min_hard_pos);
        Serial.println("stop down: " + String(_abs_pos));
        _prev_hlim_triggered = LIMIT_DOWN_PIN;
        return retval;
    } else{
        return steps_remaining;
    }
    
}




// void AbsStepper::setCurPosAtPos(){
//     AbsStepper::setCurPosAtPos(long(0));
// }

// Setup step count on current position
void AbsStepper::setCurPosAtPos(long abs_step){
    // Serial.println("_abs_pos setter");
    if (getCurrentState() == State::STOPPED){
        _abs_pos = abs_step;
        Serial.println("_abs_pos SET: " + String(_abs_pos));
    }
    
}

void AbsStepper::setCurPosAtAdeg(double abs_deg){
    // setCurPosAtPos(long(abs_deg * _step_per_deg));
    setCurPosAtPos(convertAdeg2Pos(abs_deg));
}

void AbsStepper::setCurPosAtJdeg(double jdeg){
    // setCurPosAtPos(long(jdeg * stepper2joint_ratio * _step_per_deg));
    setCurPosAtPos(convertJdeg2Pos(jdeg));
}



// // Update absolut current step count when updating [step_count]
// void AbsStepper::calcStepPulse(void){
//     DRV8825::calcStepPulse();
//     // calcStepPulse will always increase step_count, edit our abs_step as well
//     _abs_pos += getDirection();      // step_count is always positive
//     // Serial.print("y");

// }

// Update absolut current step count when updating [step_count]
void AbsStepper::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    steps_remaining--;
    step_count++;
    _abs_pos += getDirection();      // RANDEL: since step_count is always positive

    if (profile.mode == LINEAR_SPEED){
        switch (getCurrentState()){
        case ACCELERATING:
            if (step_count < steps_to_cruise){
                step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
                rest = (step_count < steps_to_cruise) ? (2*step_pulse+rest) % (4*step_count+1) : 0;
            } else {
                // The series approximates target, set the final value to what it should be instead
                step_pulse = cruise_step_pulse;
            }
            break;

        case DECELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(-4*steps_remaining+1);
            rest = (2*step_pulse+rest) % (-4*steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}


// Setup movement
uint8_t AbsStepper::startAbsMove_(long abs_dstep, long time, uint8_t exec_on_soft_limit){
    // Serial.println("d " + String(abs_dstep));
    if(_use_soft_limits){
        if(abs_dstep >= _max_soft_pos){                        // go to max step
            if(exec_on_soft_limit){
                startMove(_max_soft_pos - _abs_pos, time);
                return TRUE;
            } else{
                return FALSE;
            }
        } else if(abs_dstep <= _min_soft_pos){                 // go to min step
            if(exec_on_soft_limit){
                startMove(_min_soft_pos - _abs_pos, time);
                return TRUE;
            } else{
                return FALSE;
            }
        } else{                                             // dstep within limits, exec it
            startMove(abs_dstep - _abs_pos, time);
            return TRUE;
        }
    } else{                                                 // DON'T enforce soft limits
        // abs_dstep = desire step on absolute count
        startMove(abs_dstep - _abs_pos, time);
        return TRUE;
    }
    
}

uint8_t AbsStepper::startAbsMove(long abs_dstep, long time){
    return startAbsMove_(abs_dstep, time, FALSE);
}

uint8_t AbsStepper::startAbsMove0(long abs_dstep, long time){
    return startAbsMove_(abs_dstep, time, FALSE);
}

uint8_t AbsStepper::startAbsMove1(long abs_dstep, long time){
    return startAbsMove_(abs_dstep, time, TRUE);
}


uint8_t AbsStepper::startAbsRotate_(double ddeg, long time, uint8_t exec_on_soft_limit){
    return startAbsMove_(convertAdeg2Pos(ddeg), time, exec_on_soft_limit);
}

uint8_t AbsStepper::startAbsRotate(double ddeg, long time){
    return startAbsRotate_(ddeg, time, FALSE);
}

uint8_t AbsStepper::startAbsRotate0(double ddeg, long time){
    return startAbsRotate_(ddeg, time, FALSE);
}

uint8_t AbsStepper::startAbsRotate1(double ddeg, long time){
    return startAbsRotate_(ddeg, time, TRUE);
}


uint8_t AbsStepper::startJointRotate_(double jdeg, long time, uint8_t exec_on_soft_limit){
    // return startAbsRotate_(jdeg*stepper2joint_ratio, time, exec_on_soft_limit);
    // Serial.println("jd " + String(convertJdeg2Adeg(jdeg)));
    return startAbsRotate_(convertJdeg2Adeg(jdeg), time, exec_on_soft_limit);
}

uint8_t AbsStepper::startJointRotate(double jdeg, long time){
    return startJointRotate_(jdeg, time, FALSE);
}
uint8_t AbsStepper::startJointRotate0(double jdeg, long time){
    return startJointRotate_(jdeg, time, FALSE);
}

uint8_t AbsStepper::startJointRotate1(double jdeg, long time){
    return startJointRotate_(jdeg, time, TRUE);
}




// Setup SOFT limits
void AbsStepper::setSoftLimAsPos(long min, long max, uint8_t use_soft_limits_){
    _min_soft_pos = min;
    _max_soft_pos = max;
    _use_soft_limits = use_soft_limits_;
}

void AbsStepper::setSoftLimAsAdeg(double min, double max, uint8_t use_soft_limits_){
    setSoftLimAsPos(convertAdeg2Pos(min), convertAdeg2Pos(max), use_soft_limits_);
}

void AbsStepper::setSoftLimAsJdeg(double min, double max, uint8_t use_soft_limits_){
    setSoftLimAsPos(convertJdeg2Pos(min), convertJdeg2Pos(max), use_soft_limits_);
}



void AbsStepper::setJoint2StepperRatio(double j_gear_teeth_count, double s_gear_teeth_count){
    if(j_gear_teeth_count == 0.0 || s_gear_teeth_count == 0.0){
        Serial.println("***** ERROR [setJoint2StepperRatio]: arguments cannot be 0.");
        return;
    }

    // Serial.println("*** Adjusting limits to NEW joint ratios!");
    // before changing the gear ratios, get old values for limits 1st with old gear ratios
    double temp_jhlim_up = convertPos2Jdeg(_lim_max_hard_pos);
    double temp_jhlim_down = convertPos2Jdeg(_lim_min_hard_pos);
    double temp_jslim_up = convertPos2Jdeg(_min_soft_pos);
    double temp_jslim_down = convertPos2Jdeg(_max_soft_pos);

    joint2stepper_ratio = s_gear_teeth_count/j_gear_teeth_count;
    stepper2joint_ratio = j_gear_teeth_count/s_gear_teeth_count;

    // change all limits to reflect the gear ratio change
    setHardLimAsPos(convertJdeg2Pos(temp_jhlim_up), LIMIT_UP_PIN);
    setHardLimAsPos(convertJdeg2Pos(temp_jhlim_down), LIMIT_DOWN_PIN);
    _min_soft_pos = convertJdeg2Pos(temp_jslim_up);
    _max_soft_pos = convertJdeg2Pos(temp_jslim_down);
}

long AbsStepper::nextAction(void){
    if (univ_hard_lim == LIMIT_PIN_MAX_AS_UNIV){
        // LIMIT_PIN_MAX_AS_UNIV guarantees limit switch pin is in use
        short cur_lim_state = (digitalRead(_limitsw_up_pin) == limitsw_active_state);        // bool, 1 for limit triggered
        
        // to activate limit callbacks, previous state should be NON-TRIGGERED
        if(cur_lim_state && !_prev_hlim_state && (getDirection() > 0)){     
            limitUpActivatedCB1();
            // _prev_hlim_triggered = LIMIT_UP_PIN;
        } else if (cur_lim_state && !_prev_hlim_state && (getDirection() < 0)){
            limitDownActivatedCB1();
            // _prev_hlim_triggered = LIMIT_DOWN_PIN;
        } else if (cur_lim_state && _prev_hlim_state && (_prev_hlim_triggered==LIMIT_UP_PIN)){
            limitUpActivatedCB1();
        } else if (cur_lim_state && _prev_hlim_state && (_prev_hlim_triggered==LIMIT_DOWN_PIN)){
            limitDownActivatedCB1();
        }
        _prev_hlim_state = cur_lim_state;
    }
    else if(univ_hard_lim == LIMIT_PINS_SEPARATE){
        // poll up and down lim pins separately
        if(_limitsw_up_pin != SHRT_MIN){
            if(digitalRead(_limitsw_up_pin) == limitsw_active_state){
                limitUpActivatedCB1();
            }
        }
        // poll limit down switch
        if(_limitsw_down_pin != SHRT_MIN){
            if(digitalRead(_limitsw_down_pin) == limitsw_active_state){
                limitDownActivatedCB1();
            }
    }

    }

    return DRV8825::nextAction();
    
}


void AbsStepper::printStats(){
    Serial.println("\n****************"
                   "\n_abs_cstep: " + String(_abs_pos) + " (" + String(getCurJdeg()) + "jdeg)" +
                   "\nmicrosteps: " + String(microsteps) +
                   "\nstepper2joint_ratio: " + String(stepper2joint_ratio) +
                   "\njoint2stepper_ratio: " + String(joint2stepper_ratio) +
                   "\nstep_per_deg: " + String(_step_per_deg) +
                   "\ndeg_per_step: " + String(_deg_per_step) +
                   "\n_lim_min_hard_pos: " + String(_lim_min_hard_pos) + " (" + String(convertPos2Jdeg(_lim_min_hard_pos)) + "jdeg)" +
                   "\n_lim_max_hard_pos: " + String(_lim_max_hard_pos) + " (" + String(convertPos2Jdeg(_lim_max_hard_pos)) + "jdeg)" +
                   "\n_min_soft_pos: " + String(_min_soft_pos) + " (" + String(convertPos2Jdeg(_min_soft_pos)) + "jdeg)" +
                   "\n_max_soft_pos: " + String(_max_soft_pos) + " (" + String(convertPos2Jdeg(_max_soft_pos)) + "jdeg)" +
                   "\n****************\n"
                   );
}