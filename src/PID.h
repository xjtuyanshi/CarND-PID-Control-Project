#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>
using namespace std;
class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError(double cte);

  /**
   * get steering value to control the car.
   * @output steering value [-1,1]
   */
  double GetSteerValue();
  /**
   * Reset errors for next iter
   */
  void ResetParameters();

  /**
   * Restart the simulator so twiddle can rerun simulation
   */
  void RestartSimulator(uWS::WebSocket<uWS::SERVER> ws);

  /**
   * Set WS in PID so twiddle function can restart Websocket
   */
  void SetWS(uWS::WebSocket<uWS::SERVER> ws);
  /**
   * Twiddle Function
   */
  void Twiddle();
  /*void SetPIDPar(double twiddle_p[3]);*/
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  // previous cte 
  double prev_cte;

  // ------following variables is for twiddle------
  //websocket
  uWS::WebSocket<uWS::SERVER> webSocket;
  //dp intial values
  double dp[3] = {1,1,1};
  //p intial values
  double p[3] = {0,0,0};
  //best values
  double best_p[3];

  int max_iter = 1000;// max iteration for each test
  int iter = 0; //iteration index
 
  
 
  double total_error = 0; //accumlated error
  double best_err = 9999999;
  int index_PID_par = 0;
  bool par_test_run = true; // flag for if this is test run before (if err < best_err else ...etc)
  bool flag_after_worse_run = false; 
  bool stop_twiddle = false;
public:
  int index_collection = 0; // index of error collection steps
   //int start_error_collection = 100; // after n conmunications with simulator -start to get error 
  //int end_error_collection = 300; // after n conmunications with simulator - end  getting error -twiddle start (min)
};

#endif  // PID_H