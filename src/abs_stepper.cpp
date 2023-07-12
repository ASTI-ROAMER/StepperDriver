
#include "abs_stepper.h"



// void AbsStepper::setCurPosAsAbsStep(){
//     AbsStepper::setCurPosAsAbsStep(long(0));
// }

// Setup step count on current position
void AbsStepper::setCurPosAsAbsStep(long abs_step){
    if (getCurrentState() == State::STOPPED){
        _abs_cstep = abs_step;
    }
    
}

void AbsStepper::setCurPosAsAbsDeg(double getAbsCurDeg){
    setCurPosAsAbsStep(long(getAbsCurDeg * step_per_deg));
}


// Update absolut current step count when updating [step_count]
void AbsStepper::calcStepPulse(void){
    DRV8825::calcStepPulse();
    // calcStepPulse will always increase step_count, edit our abs_step as well
    _abs_cstep += getDirection();      // step_count is always positive
    // Serial.print("y");

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
    return startAbsMove_(long(ddeg*step_per_deg), time, exec_on_soft_limit);
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


// Setup limits
void AbsStepper::setAbsStepSoftLimits(long min, long max, uint8_t use_soft_limits_){
    _min_astep = min;
    _max_astep = max;
    use_soft_limits = use_soft_limits_;
}

void AbsStepper::setAbsDegSoftLimits(double min, double max, uint8_t use_soft_limits_){
    setAbsStepSoftLimits(long(min * step_per_deg), long(max * step_per_deg), use_soft_limits_);
}

// long AbsStepper::nextAction(void){
//     DRV8825::nextAction();
    
// }
