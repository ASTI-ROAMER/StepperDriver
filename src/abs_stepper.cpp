
#include "abs_stepper.h"


void AbsStepper::setLimitSWPin(short limit_pin_, short active_state, LimitSWMethod method){
    limit_sw_pin = limit_pin_;
    limit_sw_active_state = active_state;
    
    pinMode(limit_pin_, INPUT_PULLUP);

    // if(method == POLL){

    // }
}

void AbsStepper::setLimitSWAbsStep(long a_step){
    limit_sw_reset_astep = a_step;
}

void AbsStepper::setLimitSWAbsDeg(double ddeg){
    limit_sw_reset_astep = ddeg * step_per_deg;
}

void AbsStepper::setLimitSWJointDeg(double jdeg){
    limit_sw_reset_astep = jdeg * stepper2joint_ratio * step_per_deg;
}


long AbsStepper::limitActivatedCB1(){
    long retval=stop();
    
    // Set current position to set value
    // _abs_cstep = limit_sw_reset_astep;
    setCurPosAsAbsStep(limit_sw_reset_astep);
    Serial.println("stop: " + String(_abs_cstep));

    return retval;
}




// void AbsStepper::setCurPosAsAbsStep(){
//     AbsStepper::setCurPosAsAbsStep(long(0));
// }

// Setup step count on current position
void AbsStepper::setCurPosAsAbsStep(long abs_step){
    if (getCurrentState() == State::STOPPED){
        _abs_cstep = abs_step;
    }
    
}

void AbsStepper::setCurPosAsAbsDeg(double abs_deg){
    setCurPosAsAbsStep(long(abs_deg * step_per_deg));
}

void AbsStepper::setCurPosAsJointDeg(double jdeg){
    setCurPosAsAbsStep(long(jdeg * stepper2joint_ratio * step_per_deg));
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
    if(use_soft_limits){
        if(abs_dstep >= _max_astep){                        // go to max step
            if(exec_on_soft_limit){
                startMove(_max_astep - _abs_cstep, time);
                return TRUE;
            } else{
                return FALSE;
            }
        } else if(abs_dstep <= _min_astep){                 // go to min step
            if(exec_on_soft_limit){
                startMove(_min_astep - _abs_cstep, time);
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
    return startAbsMove_((long) (ddeg*step_per_deg), time, exec_on_soft_limit);
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
    return startAbsRotate_(jdeg*stepper2joint_ratio, time, exec_on_soft_limit);
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




// Setup limits
void AbsStepper::setAbsStepSoftLimits(long min, long max, uint8_t use_soft_limits_){
    _min_astep = min;
    _max_astep = max;
    use_soft_limits = use_soft_limits_;
}

void AbsStepper::setAbsDegSoftLimits(double min, double max, uint8_t use_soft_limits_){
    setAbsStepSoftLimits(long(min * step_per_deg), long(max * step_per_deg), use_soft_limits_);
}

void AbsStepper::setJointSoftLimits(double min, double max, uint8_t use_soft_limits_){
    setAbsStepSoftLimits(long(min * stepper2joint_ratio * step_per_deg), long(max * stepper2joint_ratio * step_per_deg), use_soft_limits_);
}



void AbsStepper::setJoint2StepperRatio(double s_moved_angle, double j_moved_angle){
    joint2stepper_ratio = j_moved_angle/s_moved_angle;
    stepper2joint_ratio = s_moved_angle/j_moved_angle;
}

long AbsStepper::nextAction(void){
    // poll limit switch
    if(limit_sw_pin != SHRT_MIN){
        if(digitalRead(limit_sw_pin) == limit_sw_active_state){
            limitActivatedCB1();
        }
    }

    DRV8825::nextAction();
    
}
