#ifndef ABS_STEPPER_H
#define ABS_STEPPER_H
#include <Arduino.h>
#include "DRV8825.h"

class AbsStepper : public DRV8825 {
protected:
    long _abs_cstep;                        // current step on absolute counts
    const double deg_per_step;              // initialize upon class construction (initialization list)
    const double step_per_deg;
    uint8_t use_soft_limits=FALSE;          // whether we use _min_astep/_max_astep to limit steps
    long _min_astep, _max_astep;            // software absolute step count limit (soft limits)

    void calcStepPulse(void) override;      // extend this so we can have absolute position


public:
    AbsStepper(short steps, short dir_pin, short step_pin)
    :DRV8825(steps, dir_pin, step_pin), _abs_cstep(0), deg_per_step(360.0 / motor_steps), step_per_deg(motor_steps / 360.0)
    {}

    AbsStepper(short steps, short dir_pin, short step_pin, short enable_pin)
    :DRV8825(steps, dir_pin, step_pin, enable_pin), _abs_cstep(0), deg_per_step(360.0 / motor_steps), step_per_deg(motor_steps / 360.0)
    {}

    /*
    * A4988-DRV8825 Compatibility map: MS1-MODE0, MS2-MODE1, MS3-MODE2
    */
    AbsStepper(short steps, short dir_pin, short step_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    :DRV8825(steps, dir_pin, step_pin, mode0_pin, mode1_pin, mode2_pin), _abs_cstep(0), deg_per_step(360.0 / motor_steps), step_per_deg(motor_steps / 360.0)
    {}

    AbsStepper(short steps, short dir_pin, short step_pin, short enable_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    :DRV8825(steps, dir_pin, step_pin, enable_pin, mode0_pin, mode1_pin, mode2_pin), _abs_cstep(0), deg_per_step(360.0 / motor_steps), step_per_deg(motor_steps / 360.0)
    {}

    // Resets position WITHOUT MOVING
    // void setCurPosAsAbsStep();
    void setCurPosAsAbsStep(long abs_step);       // Sets abs_cstep to given number, default 0
    void setCurPosAsAbsDeg(double getAbsCurDeg);

    // absolute movement (non-blocking)
    // By step count:
    // - IFF soft limits are enforced:
    // --exec_on_soft_limit==TRUE: if the desired step exceeds the limit, will CONTINUE to move until limit is reached.
    // --exec_on_soft_limit==FALSE: if the desired step exceeds the limit, will NOT MOVE the motor
    uint8_t startAbsMove_(long abs_dstep, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startAbsMove(long abs_dstep, long time=0);          // startAbsMove_ with exec_on_soft_limit=FALSE
    uint8_t startAbsMove0(long abs_dstep, long time=0);         // startAbsMove_ with exec_on_soft_limit=FALSE
    uint8_t startAbsMove1(long abs_dstep, long time=0);         // startAbsMove_ with exec_on_soft_limit=TRUE

    // By degrees:
    uint8_t startAbsRotate_(double ddeg, long time=0, uint8_t exec_on_soft_limit=FALSE);
    uint8_t startAbsRotate(double ddeg, long time=0);           // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate0(double ddeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate1(double ddeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=TRUE


    // setup absolute step/angle software limits, use_soft_limits_=TRUE will enforce the limits
    void setAbsStepSoftLimits(long min, long max, uint8_t use_soft_limits_=TRUE);
    void setAbsDegSoftLimits(double min, double max, uint8_t use_soft_limits_=TRUE);



    // // Move motor to detect limit switch
    // short NestMotor();
    // short NestMotor(double angle_on_nest=0.0);

    // long nextAction(void);

    // Getters
    const long& getAbsCurStep() const {return _abs_cstep;}
    const double& getAbsCurDeg() const {return _abs_cstep * deg_per_step;}

    const double& getStepPerDeg() const {return step_per_deg;}
    
};
#endif // ABS_STEPPER_H
