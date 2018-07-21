#ifndef PID_H
#define PID_H

  typedef enum
  {
	  RUN = 0,
	  PROCEED_1,
	  PROCEED_2,
	  PROCEED_3,
	  DONE
  } TwiddleState_ten;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  TwiddleState_ten TwiddleState;
  TwiddleState_ten LastTwiddleState;
  
  int TwiddleRunningSteps;
  
  int TwiddleRunningCounter;
  int AttrCounter;
  
  double TwiddleBestErr;
  double TwiddleLastErr;
  
  double Tolerant;
  double K[3];
  double dk[3];


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  void TwiddleInit(int RunningSteps, TwiddleState_ten InitState, double Tolerant);
  
  bool Twiddle(float cte);
  
  bool IsTwiddleNeeded();
  
private:

  void TwiddleRunningState(float cte);
  
  bool TwiddleProceed1();
  
  bool TwiddleProceed2();
  
  void TwiddleProceed3();
  
  float TwiddleSumDiff();
};

#endif /* PID_H */
