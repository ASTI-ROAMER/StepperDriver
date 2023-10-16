#ifndef ABS_STEPPER_H
#define ABS_STEPPER_H
#include <Arduino.h>
#include "DRV8825.h"
#include <limits.h>

/**
 * @brief Absolute positioning and joint wrapper for stepper driver
 * 
 * Uses 2 types of limit:
 *  Soft limits: Required. Implemented by software. Movement will not exceed this unless explicitly commanded.
 *  Hard limits: Optional. Implemented by limit switches. Triggering the corresponding limit switch will forcefully set the current position the the preset value of the hard limit.
 * 
 * 
 * Terms:
 *  pos - absolute step position tracked by moving the stepper
 *  adeg - absolute position in degrees (rotation), the position in degrees using degree per step value
 *  lim_jdeg - joint position in terms of joint degrees (rotation), the position OF THE JOINT in degrees using joint gear ration and degree per step values.
 *  
 */
class AbsStepper : public DRV8825 {
public:
    enum LimitStatus {NORMAL, ERROR};
    enum LimitSWMethod {POLL, INTERRUPT};
    enum LimitSWPinSelect {LIMIT_NONE, LIMIT_DOWN_PIN, LIMIT_UP_PIN};
    enum LimitSWUniversalPin {LIMIT_PINS_UNAVAILABLE, LIMIT_PINS_SEPARATE, LIMIT_PIN_MIN_AS_UNIV, LIMIT_PIN_MAX_AS_UNIV};

    short limitsw_active_state=LOW;             // whether high or low is the trigger signal
    LimitSWUniversalPin univ_hard_lim=LIMIT_PINS_SEPARATE;
    // short limitsw_up_active_state=LOW;        
    // short limitsw_down_active_state=LOW;

protected:
    long _abs_pos;                          // position, the current step on absolute counts
    double _deg_per_step;                    // values relating the step count to rotational position
    double _step_per_deg;

    // Limit switch pins
    short _limitsw_up_pin=SHRT_MIN;              // limit switch pin# (arduino)assume SHRT_MIN means undefined pin (not used)
    short _limitsw_down_pin=SHRT_MIN;

    

    uint8_t _use_soft_limits=FALSE;          // flag for using _min_soft_pos/_max_soft_pos to limit steps
    short _prev_hlim_state=limitsw_active_state;
    LimitSWPinSelect _prev_hlim_triggered=LIMIT_NONE;
    long _min_soft_pos, _max_soft_pos;      // software absolute step count limit (soft limits)
    long _lim_min_hard_pos=0L;
    long _lim_max_hard_pos=10L;             // absolute step value will be this when limit sw is triggered

    double joint2stepper_ratio = 1.0;       // values for relating adeg to lim_jdeg
    double stepper2joint_ratio = 1.0;

    void calcStepPulse(void) override;      // extend this so we can have absolute position


public:
    AbsStepper(short steps, short dir_pin, short step_pin)
    :DRV8825(steps, dir_pin, step_pin), _abs_pos(0), _deg_per_step(360.0 / motor_steps), _step_per_deg(motor_steps / 360.0)
    {}

    AbsStepper(short steps, short dir_pin, short step_pin, short enable_pin)
    :DRV8825(steps, dir_pin, step_pin, enable_pin), _abs_pos(0), _deg_per_step(360.0 / motor_steps), _step_per_deg(motor_steps / 360.0)
    {}

    /*
    * A4988-DRV8825 Compatibility map: MS1-MODE0, MS2-MODE1, MS3-MODE2
    */
    AbsStepper(short steps, short dir_pin, short step_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    :DRV8825(steps, dir_pin, step_pin, mode0_pin, mode1_pin, mode2_pin), _abs_pos(0), _deg_per_step(360.0 / motor_steps), _step_per_deg(motor_steps / 360.0)
    {}

    AbsStepper(short steps, short dir_pin, short step_pin, short enable_pin, short mode0_pin, short mode1_pin, short mode2_pin)
    :DRV8825(steps, dir_pin, step_pin, enable_pin, mode0_pin, mode1_pin, mode2_pin), _abs_pos(0), _deg_per_step(360.0 / motor_steps), _step_per_deg(motor_steps / 360.0)
    {}


    // Limit switch related, activating the limit switch WILL STOP the motor
    
    void setLimitSwPin(short pin_, long lim_pos, LimitSWPinSelect pin_select=LIMIT_UP_PIN, short active_state=LOW, LimitSWMethod method=POLL);    // sets up upper limit switch
    // overload for using double (specified in joint angle, after gear ratio)
    void setLimitSwPin(short pin_, double lim_jdeg, LimitSWPinSelect pin_select=LIMIT_UP_PIN, short active_state=LOW, LimitSWMethod method=POLL);
    // Set universal hard limit pin
    void setHardLimUnivPin(short pin_, long min_pos, long max_pos, short active_state=LOW, LimitSWMethod method=POLL);
    void setHardLimUnivPin(short pin_, double min_jdeg, double max_jdeg, short active_state=LOW, LimitSWMethod method=POLL);
    
    void setHardLimAsPos(long lim_pos=0L, LimitSWPinSelect pin_select=LIMIT_UP_PIN);

    void setHardLimUpPin(short pin_, long lim_pos, short active_state=LOW, LimitSWMethod method=POLL);        // set limit switch pin
    void setHardLimUpPin(short pin_, double lim_jdeg, short active_state=LOW, LimitSWMethod method=POLL); 
    void setHardLimUpAsPos(long lim_pos=0L);                                                   // set abs step when limit is triggered
    void setHardLimUpAsAdeg(double lim_adeg=0.0);
    void setHardLimUpAsJdeg(double lim_jdeg=0.0);

    void setHardLimDownPin(short pin_, long lim_pos, short active_state=LOW, LimitSWMethod method=POLL);
    void setHardLimDownPin(short pin_, double lim_jdeg, short active_state=LOW, LimitSWMethod method=POLL);
    void setHardLimDownAsPos(long lim_pos=0L); 
    void setHardLimSWDownAsAdeg(double lim_adeg=0.0);
    void setHardLimDownAsJdeg(double lim_jdeg=0.0);

    

    long limitUpActivatedCB1(void);                             // called when limit is triggered in POLL mode
    long limitDownActivatedCB1(void);
    // long (*_limitCB)(void);                 // function pointer to CB when limit sw is activated (point it to poll/interrupt)


    // Resets position WITHOUT MOVING
    // void setCurPosAtPos();
    void setCurPosAtPos(long abs_step);       // Sets abs_cstep to given number, default 0
    void setCurPosAtAdeg(double abs_deg);
    void setCurPosAtJdeg(double lim_jdeg);

    // absolute movement (non-blocking)
    // By step count:
    // - IFF soft limits are enforced:
    // --exec_on_soft_limit==TRUE: if the desired step exceeds the limit, will CONTINUE to move until limit is reached.
    // --exec_on_soft_limit==FALSE: if the desired step exceeds the limit, will NOT MOVE the motor
    uint8_t startAbsMove_(long d_pos, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startAbsMove(long d_pos, long time=0);          // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startAbsMove0(long d_pos, long time=0);         // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startAbsMove1(long d_pos, long time=0);         // exec_on_soft_limit=TRUE, move even if invalid (up to valid position only)

    // By degrees:
    uint8_t startAbsRotate_(double lim_adeg, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startAbsRotate(double lim_adeg, long time=0);           // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate0(double lim_adeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=FALSE
    uint8_t startAbsRotate1(double lim_adeg, long time=0);          // startAbsRotate_ with exec_on_soft_limit=TRUE

    // By joint degrees:
    uint8_t startJointRotate_(double lim_jdeg, long time=0, uint8_t exec_on_soft_limit=FALSE);
    // Convinience methods
    uint8_t startJointRotate(double lim_jdeg, long time=0);         // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startJointRotate0(double lim_jdeg, long time=0);        // exec_on_soft_limit=FALSE, don't move at all if invalid position
    uint8_t startJointRotate1(double lim_jdeg, long time=0);        // exec_on_soft_limit=TRUE, move even if invalid (up to valid position only)


    // setup absolute step/angle software limits, use_soft_limits_=TRUE will enforce the limits
    void setSoftLimAsPos(long min, long max, uint8_t use_soft_limits_=TRUE);
    void setSoftLimAsAdeg(double min, double max, uint8_t use_soft_limits_=TRUE);
    void setSoftLimAsJdeg(double min, double max, uint8_t use_soft_limits_=TRUE);

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
    inline double convertJdeg2Adeg(double lim_jdeg){return (double)(lim_jdeg * stepper2joint_ratio);}
    inline long convertJdeg2Pos(double lim_jdeg) {return (long)(lim_jdeg * stepper2joint_ratio * _step_per_deg);}
    inline long convertAdeg2Pos(double adeg){return (long)(adeg * _step_per_deg);}
    inline double convertAdeg2Jdeg(double adeg){return (double)(adeg * joint2stepper_ratio);}
    inline double convertPos2Adeg(long pos){return (double)(pos * _deg_per_step);}
    inline double convertPos2Jdeg(long pos){return (double)(pos * _deg_per_step * joint2stepper_ratio);}
    



    // // Move motor to detect limit switch
    // short NestMotor();
    // short NestMotor(double angle_on_nest=0.0);

    long nextAction(void);


    // Getters and convinences
    const long& getCurPos() const {return _abs_pos;}          //so that _abs_pos will not be modified
    const double& getStepPerDeg() const {return _step_per_deg;}
    const double& getDegPerStep() const {return _deg_per_step;}

    double getCurAdeg() const {return _abs_pos * _deg_per_step;}
    double getCurJdeg() const {return _abs_pos * _deg_per_step * joint2stepper_ratio;}

    void printStats(void);

    
    
};
#endif // ABS_STEPPER_H
