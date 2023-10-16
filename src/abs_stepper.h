#ifndef ABS_STEPPER_H
#define ABS_STEPPER_H
#include <Arduino.h>
#include "DRV8825.h"
#include <limits.h>

class AbsStepper : public DRV8825 {
protected:
    long _abs_cstep;                        // current step on absolute counts
    double deg_per_step;              // initialize upon class construction (initialization list)
    double step_per_deg;
    uint8_t use_soft_limits=FALSE;          // whether we use _min_soft_astep/_max_soft_astep to limit steps
    long _min_soft_astep, _max_soft_astep;            // software absolute step count limit (soft limits)

    double joint2stepper_ratio = 1.0;          
    double stepper2joint_ratio = 1.0;

    

    void calcStepPulse(void) override;      // extend this so we can have absolute position


public:
    enum LimitStatus {NORMAL, ERROR};
    enum LimitSWMethod {POLL, INTERRUPT};
    enum LimitSWPinSelect {LIMIT_DOWN_PIN, LIMIT_UP_PIN};

    // Limit switch pins
    short limitsw_up_pin=SHRT_MIN;              // limit switch pin# (arduino)assume SHRT_MIN means undefined pin (not used)
    short limitsw_down_pin=SHRT_MIN;

    short limitsw_up_active_state=LOW;         // whether high or low is the trigger signal
    short limitsw_down_active_state=LOW;

    long limitsw_up_reset_astep=10L;                // absolute step value will be this when limit sw is triggered
    long limitsw_down_reset_astep=0L;

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


    // Limit switch related, activating the limit switch WILL STOP the motor
    void setLimitSwPin(short pin_, long lim_abs_step, LimitSWPinSelect pin_select=LIMIT_UP_PIN, short active_state=LOW, LimitSWMethod method=POLL);    // sets up upper limit switch
    // overload for using double (specified in joint angle, after gear ratio)
    void setLimitSwPin(short pin_, double jdeg, LimitSWPinSelect pin_select=LIMIT_UP_PIN, short active_state=LOW, LimitSWMethod method=POLL);
    void setLimitSWAbsStep(long a_step=0L, LimitSWPinSelect pin_select=LIMIT_UP_PIN);

    void setLimitSwUpPin(short pin_, long lim_abs_step, short active_state=LOW, LimitSWMethod method=POLL);        // set limit switch pin
    void setLimitSwUpPin(short pin_, double jdeg, short active_state=LOW, LimitSWMethod method=POLL); 
    void setLimitSWUpAbsStep(long a_step=0L);                                                   // set abs step when limit is triggered
    void setLimitSWUpAbsDeg(double ddeg=0.0);
    void setLimitSWUpJointDeg(double jdeg=0.0);

    void setLimitSwDownPin(short pin_, long lim_abs_step, short active_state=LOW, LimitSWMethod method=POLL);
    void setLimitSwDownPin(short pin_, double jdeg, short active_state=LOW, LimitSWMethod method=POLL);
    void setLimitSWDownAbsStep(long a_step=0L); 
    void setLimitSWDownAbsDeg(double ddeg=0.0);
    void setLimitSWDownJointDeg(double jdeg=0.0);

    long limitUpActivatedCB1(void);                             // called when limit is triggered in POLL mode
    long limitDownActivatedCB1(void);
    // long (*_limitCB)(void);                 // function pointer to CB when limit sw is activated (point it to poll/interrupt)


    // Resets position WITHOUT MOVING
    // void setCurPosAsAbsStep();
    void setCurPosAsAbsStep(long abs_step);       // Sets abs_cstep to given number, default 0
    void setCurPosAsAbsDeg(double abs_deg);
    void setCurPosAsJointDeg(double jdeg);

    // absolute movement (non-blocking)
    // By step count:
    // - IFF soft limits are enforced:
    // --exec_on_soft_limit==TRUE: if the desired step exceeds the limit, will CONTINUE to move until limit is reached.
    // --exec_on_soft_limit==FALSE: if the desired step exceeds the limit, will NOT MOVE the motor
    uint8_t startAbsMove_(long abs_dstep, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startAbsMove(long abs_dstep, long time=0);          // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startAbsMove0(long abs_dstep, long time=0);         // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startAbsMove1(long abs_dstep, long time=0);         // exec_on_soft_limit=TRUE, move even if invalid (up to valid position only)

    // By degrees:
    uint8_t startAbsRotate_(double ddeg, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startAbsRotate(double ddeg, long time=0);           // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate0(double ddeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate1(double ddeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=TRUE

    // By joint degrees:
    uint8_t startJointRotate_(double jdeg, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startJointRotate(double jdeg, long time=0);         // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startJointRotate0(double jdeg, long time=0);        // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startJointRotate1(double jdeg, long time=0);        // exec_on_soft_limit=TRUE, move even if invalid (up to valid position only)


    // setup absolute step/angle software limits, use_soft_limits_=TRUE will enforce the limits
    void setAbsStepSoftLimits(long min, long max, uint8_t use_soft_limits_=TRUE);
    void setAbsDegSoftLimits(double min, double max, uint8_t use_soft_limits_=TRUE);
    void setJointSoftLimits(double min, double max, uint8_t use_soft_limits_=TRUE);

    /**
     * @brief joint interface - joints may be geared, this takes care of that.
     * Calculates and sets gearing ratio given a set movement of the stepper motor WRT the joint.
     * -- joint angle / stepper angle
     * 
     * @param j_gear_teeth_count joint gear teeth count OR  can also be the angle movement observed on the stepper side WHEN the joint is moved by a set angle
     * @param s_gear_teeth_count stepper gear teeth count OR can also be the angle movement observed on the joint side when the stepper is moved by a set angle
     */
    void setJoint2StepperRatio(double j_gear_teeth_count, double s_gear_teeth_count);



    // convertion functions
    // convert joint angle to absolute stepper angle
    inline double convertJdeg2Adeg(double jdeg){return (double)(jdeg * stepper2joint_ratio);}
    inline long convertJdeg2Astep(double jdeg) {return (long)(jdeg * stepper2joint_ratio * step_per_deg);}
    inline long convertAdeg2Astep(double adeg){return (long)(adeg * step_per_deg);}
    inline double convertAdeg2Jdeg(double adeg){return (double)(adeg * joint2stepper_ratio);}
    inline double convertAstep2Adeg(long astep){return (double)(astep * deg_per_step);}
    inline double convertAstep2Jdeg(long astep){return (double)(astep * deg_per_step * joint2stepper_ratio);}
    



    // // Move motor to detect limit switch
    // short NestMotor();
    // short NestMotor(double angle_on_nest=0.0);

    long nextAction(void);


    // Getters and convinences
    const long& getAbsCurStep() const {return _abs_cstep;}          //so that _abs_cstep will not be modified
    const double& getStepPerDeg() const {return step_per_deg;}
    const double& getDegPerStep() const {return deg_per_step;}

    double getAbsCurDeg() const {return _abs_cstep * deg_per_step;}
    double getJointCurDeg() const {return _abs_cstep * deg_per_step * joint2stepper_ratio;}

    void printStats(void);

    
    
};
#endif // ABS_STEPPER_H
