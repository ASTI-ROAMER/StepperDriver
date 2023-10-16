
#include "abs_stepper.h"

// limit (HARD) switch setup 
void AbsStepper::setLimitSwPin(short pin_, long lim_abs_step, LimitSWPinSelect pin_select, short active_state, LimitSWMethod method){
    // Serial.println("***lim_abs_step lim up: " + String(lim_abs_step));
    if(pin_select == LIMIT_DOWN_PIN){
        limitsw_down_pin = pin_;
        limitsw_down_active_state = active_state;
    } else{
        limitsw_up_pin = pin_;
        limitsw_up_active_state = active_state;
    }
    // Serial.println("***setuppin step: " + String(lim_abs_step) + "  pin_select" + String(pin_select));
    setLimitSWAbsStep(lim_abs_step, pin_select);
    pinMode(pin_, INPUT_PULLUP);

    // if(method == POLL){

    // }
}

// Overload: use joint deg (double) instead of abs steps
void AbsStepper::setLimitSwPin(short pin_, double jdeg, LimitSWPinSelect pin_select, short active_state, LimitSWMethod method){
    // Serial.println("***JDEG lim up: " + String((float)jdeg));
    // setLimitSwPin(pin_, (long)(jdeg * stepper2joint_ratio * step_per_deg), pin_select, active_state, method);
    setLimitSwPin(pin_, convertJdeg2Astep(jdeg), pin_select, active_state, method);

}

void AbsStepper::setLimitSWAbsStep(long a_step, LimitSWPinSelect pin_select){
    if(pin_select == LIMIT_DOWN_PIN){
        if(limitsw_up_pin != SHRT_MIN){                     // there is a uppper limit switch value, so check it first before setting lower limit
            if(a_step < limitsw_up_reset_astep){            // check if lower limit is lower than upper limit
                limitsw_down_reset_astep = a_step;
                Serial.println("(lim_down < lim_up OK) Set lim down: " + String(limitsw_up_reset_astep));
            } else {
                // error - lower limit is higher than upper limit
                Serial.println("***** ERROR: Limit switch reset position error.");
            }
        } else{                                             // no upper limit, set up lower limit immediately
            limitsw_down_reset_astep = a_step;
            Serial.println("Set lim down DIRECTLY:" + String(limitsw_down_reset_astep));
        }
        
    } else{
        if(limitsw_down_pin != SHRT_MIN){                   // there is a lower limit switch value, so check it first before setting upper limit
            if(a_step > limitsw_down_reset_astep){          // check if upper limit is higher than lower limit
                // Serial.println("Setting lim up to: " + String(a_step));
                limitsw_up_reset_astep = a_step;
                Serial.println("(lim_down < lim_up OK) Set lim up: " + String(limitsw_up_reset_astep));
            } else {
                // error - upper limit is lower than lower limit
                Serial.println("***** ERROR: Limit switch reset position error.");
            }
        } else{                                             // no lower limit, set up upper limit immediately
            // Serial.println("Setting lim up to: %l" + String(a_step));
            limitsw_up_reset_astep = a_step;
            Serial.println("Set lim up DIRECTLY:" + String(limitsw_up_reset_astep));
        }
    }
}

void AbsStepper::setLimitSwUpPin(short pin_, long lim_abs_step, short active_state, LimitSWMethod method){
    // Serial.println("abs lim up: " + String(lim_abs_step));
    setLimitSwPin(pin_, lim_abs_step, LIMIT_UP_PIN, active_state, method);
}
void AbsStepper::setLimitSwUpPin(short pin_, double jdeg, short active_state, LimitSWMethod method){
    // Serial.println("jdeg lim up: " + String(jdeg));
    setLimitSwPin(pin_, jdeg, LIMIT_UP_PIN, active_state, method);
}

void AbsStepper::setLimitSWUpAbsStep(long a_step){
    setLimitSWAbsStep(a_step, LIMIT_UP_PIN);
}
void AbsStepper::setLimitSWUpAbsDeg(double ddeg){
    // setLimitSWAbsStep((long)(ddeg * step_per_deg), LIMIT_UP_PIN);
    setLimitSWAbsStep(convertAdeg2Astep(ddeg), LIMIT_UP_PIN);
}
void AbsStepper::setLimitSWUpJointDeg(double jdeg){
    // setLimitSWAbsStep((long)(jdeg * stepper2joint_ratio * step_per_deg), LIMIT_UP_PIN);
    setLimitSWAbsStep(convertJdeg2Astep(jdeg), LIMIT_UP_PIN);
}



void AbsStepper::setLimitSwDownPin(short pin_, long lim_abs_step, short active_state, LimitSWMethod method){
    setLimitSwPin(pin_, lim_abs_step, LIMIT_DOWN_PIN, active_state, method);
}
void AbsStepper::setLimitSwDownPin(short pin_, double jdeg, short active_state, LimitSWMethod method){
    setLimitSwPin(pin_, jdeg, LIMIT_DOWN_PIN, active_state, method);
}

void AbsStepper::setLimitSWDownAbsStep(long a_step){
    setLimitSWAbsStep(a_step, LIMIT_DOWN_PIN);
}
void AbsStepper::setLimitSWDownAbsDeg(double ddeg){
    // setLimitSWAbsStep((long)(ddeg * step_per_deg), LIMIT_DOWN_PIN);
    setLimitSWAbsStep(convertAdeg2Astep(ddeg), LIMIT_DOWN_PIN);
}
void AbsStepper::setLimitSWDownJointDeg(double jdeg){
    // setLimitSWAbsStep((long)(jdeg * stepper2joint_ratio * step_per_deg), LIMIT_DOWN_PIN);
    setLimitSWAbsStep(convertJdeg2Astep(jdeg), LIMIT_DOWN_PIN);
}


long AbsStepper::limitUpActivatedCB1(){
    // when limit sw is triggered, STOP ONLY when you are APPROACHING the set position of the limit sw
    if(steps_remaining > 0 && getDirection() == 1){
        long retval=stop();
        // Set current position to set value
        Serial.println("lim up: " + String(limitsw_up_reset_astep));
        setCurPosAsAbsStep(limitsw_up_reset_astep);
        Serial.println("stop up: " + String(_abs_cstep));
        return retval;
    } else{
        return steps_remaining;
    }
    
}

long AbsStepper::limitDownActivatedCB1(){
    if(steps_remaining > 0 && getDirection() == -1){
        long retval=stop();
        // Set current position to set value
        setCurPosAsAbsStep(limitsw_down_reset_astep);
        Serial.println("stop down: " + String(_abs_cstep));
        return retval;
    } else{
        return steps_remaining;
    }
    
}




// void AbsStepper::setCurPosAsAbsStep(){
//     AbsStepper::setCurPosAsAbsStep(long(0));
// }

// Setup step count on current position
void AbsStepper::setCurPosAsAbsStep(long abs_step){
    // Serial.println("_abs_cstep setter");
    if (getCurrentState() == State::STOPPED){
        _abs_cstep = abs_step;
        Serial.println("_abs_cstep SET: " + String(_abs_cstep));
    }
    
}

void AbsStepper::setCurPosAsAbsDeg(double abs_deg){
    // setCurPosAsAbsStep(long(abs_deg * step_per_deg));
    setCurPosAsAbsStep(convertAdeg2Astep(abs_deg));
}

void AbsStepper::setCurPosAsJointDeg(double jdeg){
    // setCurPosAsAbsStep(long(jdeg * stepper2joint_ratio * step_per_deg));
    setCurPosAsAbsStep(convertJdeg2Astep(jdeg));
}



// // Update absolut current step count when updating [step_count]
// void AbsStepper::calcStepPulse(void){
//     DRV8825::calcStepPulse();
//     // calcStepPulse will always increase step_count, edit our abs_step as well
//     _abs_cstep += getDirection();      // step_count is always positive
//     // Serial.print("y");

// }

// Update absolut current step count when updating [step_count]
void AbsStepper::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    steps_remaining--;
    step_count++;
    _abs_cstep += getDirection();      // RANDEL: since step_count is always positive

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
    if(use_soft_limits){
        if(abs_dstep >= _max_soft_astep){                        // go to max step
            if(exec_on_soft_limit){
                startMove(_max_soft_astep - _abs_cstep, time);
                return TRUE;
            } else{
                return FALSE;
            }
        } else if(abs_dstep <= _min_soft_astep){                 // go to min step
            if(exec_on_soft_limit){
                startMove(_min_soft_astep - _abs_cstep, time);
                return TRUE;
            } else{
                return FALSE;
            }
        } else{                                             // dstep within limits, exec it
            startMove(abs_dstep - _abs_cstep, time);
            return TRUE;
        }
    } else{                                                 // DON'T enforce soft limits
        // abs_dstep = desire step on absolute count
        startMove(abs_dstep - _abs_cstep, time);
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
    return startAbsMove_(convertAdeg2Astep(ddeg), time, exec_on_soft_limit);
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
void AbsStepper::setAbsStepSoftLimits(long min, long max, uint8_t use_soft_limits_){
    _min_soft_astep = min;
    _max_soft_astep = max;
    use_soft_limits = use_soft_limits_;
}

void AbsStepper::setAbsDegSoftLimits(double min, double max, uint8_t use_soft_limits_){
    setAbsStepSoftLimits(convertAdeg2Astep(min), convertAdeg2Astep(max), use_soft_limits_);
}

void AbsStepper::setJointSoftLimits(double min, double max, uint8_t use_soft_limits_){
    setAbsStepSoftLimits(convertJdeg2Adeg(min), convertJdeg2Adeg(max), use_soft_limits_);
}



void AbsStepper::setJoint2StepperRatio(double j_gear_teeth_count, double s_gear_teeth_count){
    if(j_gear_teeth_count == 0.0 || s_gear_teeth_count == 0.0){
        Serial.println("***** ERROR [setJoint2StepperRatio]: arguments cannot be 0.");
        return;
    }

    Serial.println("*** Adjusting limits to NEW joint ratios!");
    // before changing the gear ratios, get old values for limits 1st with old gear ratios
    double temp_jhlim_up = convertAstep2Jdeg(limitsw_up_reset_astep);
    double temp_jhlim_down = convertAstep2Jdeg(limitsw_down_reset_astep);
    double temp_jslim_up = convertAstep2Jdeg(_min_soft_astep);
    double temp_jslim_down = convertAstep2Jdeg(_max_soft_astep);

    joint2stepper_ratio = s_gear_teeth_count/j_gear_teeth_count;
    stepper2joint_ratio = j_gear_teeth_count/s_gear_teeth_count;

    // change all limits to reflect the gear ratio change
    setLimitSWAbsStep(convertJdeg2Astep(temp_jhlim_up), LIMIT_UP_PIN);
    setLimitSWAbsStep(convertJdeg2Astep(temp_jhlim_down), LIMIT_DOWN_PIN);
    _min_soft_astep = convertJdeg2Astep(temp_jslim_up);
    _max_soft_astep = convertJdeg2Astep(temp_jslim_down);
}

long AbsStepper::nextAction(void){
    // poll limit up switch
    // Serial.println("w");
    if(limitsw_up_pin != SHRT_MIN){
        if(digitalRead(limitsw_up_pin) == limitsw_up_active_state){
            limitUpActivatedCB1();
        }
    }

    // poll limit down switch
    if(limitsw_down_pin != SHRT_MIN){
        if(digitalRead(limitsw_down_pin) == limitsw_down_active_state){
            limitDownActivatedCB1();
        }
    }

    return DRV8825::nextAction();
    
}


void AbsStepper::printStats(){
    Serial.println("\n****************"
                   "\n_abs_cstep: " + String(_abs_cstep) +
                   "\nstepper2joint_ratio: " + String(stepper2joint_ratio) +
                   "\njoint2stepper_ratio: " + String(joint2stepper_ratio) +
                   "\nstep_per_deg: " + String(step_per_deg) +
                   "\ndeg_per_step: " + String(deg_per_step) +
                   "\nlimitsw_down_reset_astep: " + String(limitsw_down_reset_astep) +
                   "\nlimitsw_up_reset_astep: " + String(limitsw_up_reset_astep) +
                   "\n_min_soft_astep: " + String(_min_soft_astep) +
                   "\n_max_soft_astep: " + String(_max_soft_astep) +
                   "\n****************\n"
                   );
}