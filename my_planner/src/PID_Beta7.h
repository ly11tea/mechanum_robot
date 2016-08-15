#include "ros/ros.h"
#include <boost/thread.hpp>
#ifndef PID_Beta7_h
#define PID_Beta7_h

class PID
{


  public:

  #define AUTO	1
  #define MANUAL	0
  #define LIBRARY_VERSION	0.6

    PID();
  //commonly used functions **************************************************************************
    PID(double*, double*, double*,				// * constructor.  links the PID to the Input, Output, and 
        double, double, double);			//   Setpoint.  Initial tuning parameters are also set here

    PID(double*, double*, double*,				// * Overloaded Constructor.  if the user wants to implement
        double*, double, double, double);		//   feed-forward

    void SetMode(int Mode);			// * sets PID to either Manual (0) or Auto (non-0)

    void Compute();					// * performs the PID calculation.  it should be
										//   called every time loop() cycles. ON/OFF and
										//   calculation frequency can be set using SetMode
										//   SetSampleTime respectively

    void SetInputLimits(double, double);  //Tells the PID what 0-100% are for the Input

    void SetOutputLimits(double, double); //Tells the PID what 0-100% are for the Output


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,	// * While most users will set the tunings once in the 
                    double);				//   constructor, this function gives the user the option
										//   of changing tunings during runtime for Adaptive control

    void SetSampleTime(int);		// * sets the frequency, in Milliseconds, with which 
										//   the PID calculation is performed.  default is 1000 

    void Reset();						// * reinitializes controller internals.  automatically
										//   called on a manual to auto transition

    bool JustCalculated();			// * in certain situations, it helps to know when the PID has
										//   computed this bit will be true for one cycle after the
										//   pid calculation has occurred
    

   //Status functions allow you to query current PID constants ***************************************
    int GetMode();
    double GetINMin();
    double GetINMax();
    double GetOUTMin();
    double GetOUTMax();
    int GetSampleTime();
    double GetP_Param();
    double GetI_Param();
    double GetD_Param();


  private:

    void ConstructorCommon(double*, double*, double*,           // * code that is shared by the constructors
        double, double, double);

   //scaled, tweaked parameters we'll actually be using
    double kc;                    // * (P)roportional Tuning Parameter
    double taur;                  // * (I)ntegral Tuning Parameter
    double taud;                  // * (D)erivative Tuning Parameter

	double cof_A;
	double cof_B;
	double cof_C;

   //nice, pretty parameters we'll give back to the user if they ask what the tunings are
    double  P_Param;
    double I_Param;
    double D_Param;


    double *myInput;				// * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;				//   This creates a hard link between the variables and the 
    double *mySetpoint;			//   PID, freeing the user from having to constantly tell us
								//   what these values are.  with pointers we'll just know.

    double *myBias;				// * Pointer to the External FeedForward bias, only used 
								//   if the advanced constructor is used
    bool UsingFeedForward;		// * internal flag that tells us if we're using FeedForward or not

    unsigned long nextCompTime;    // * Helps us figure out when the PID Calculation needs to
                                  //   be performed next
                                  //   to determine when to compute next
    unsigned long tSample;       // * the frequency, in milliseconds, with which we want the
                                  //   the PID calculation to occur.
    bool inAuto;                  // * Flag letting us know if we are in Automatic or not

                                  //   the derivative required for the D term
    //float accError;              // * the (I)ntegral term is based on the sum of error over
                                  //   time.  this variable keeps track of that
    double bias;                  // * the base output from which the PID operates
    
	double Err;
	double lastErr;
	double prevErr;
	
    double inMin, inSpan;         // * input and output limits, and spans.  used convert
    double outMin, outSpan;       //   real world numbers into percent span, with which
								 //   the PID algorithm is more comfortable.

    bool justCalced;			// * flag gets set for one cycle after the pid calculates
    
    boost::mutex pid_mutex_;
    struct timeval te;
};

#endif

