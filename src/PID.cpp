#include "PID.h"
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   *  Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	prev_cte = 0;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
	p_error = cte;
	d_error = cte - prev_cte;
	i_error += cte;
	prev_cte = cte;
}

double PID::TotalError() {
  /**
   *  Calculate and return the total error
   */
	double output = -Kp * p_error - Kd * d_error - Ki * i_error;
	if (output > 1) {
		output = 1;
	}
	else if (output < -1) {
		output = -1;
	}
	return output;
}
void PID::RestartSimulator(uWS::WebSocket<uWS::SERVER> ws) {
	Init(p[0],p[1],p[2]);
	err_for_twiddle = 0;
	index_collection = 0;
	cout << "current PID values before restart:" << endl;
	cout << p[0] << "," << p[1] << "," << p[2] << endl;

	std::string msg = "42[\"reset\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}
void PID::SetWS(uWS::WebSocket<uWS::SERVER> ws) {
	webSocket = ws;


}
void PID::Twiddle() {
	// run the simulator for a while to collect errors
	if (!stop_twiddle) {
		if (index_collection >= start_error_collection) {
			err_for_twiddle += p_error * p_error;
		}
		if (index_collection > end_error_collection) {
			cout << "Iter =" << iter << endl;
			if (iter == 0) {
				best_err = err_for_twiddle;
			}
			//error collected -start to tune pid values
			cout << "sum of cte^2 =" << err_for_twiddle << endl;
			double sum_dp = dp[0] + dp[1] + dp[2];
			//reset index for PID parameters (0- P,1- I,2-D)
			if (index_PID_par == 3) {
				index_PID_par = 0;
				iter++;
			}
			if (sum_dp > 0.2 && iter < max_iter) {
				cout <<  "best error=" << best_err << endl;
				if (par_test_run) {
					p[index_PID_par] += dp[index_PID_par];
					cout<<"this iter:" <<iter<<",first run, tunning dp[" << index_PID_par << "] by +1* to " << dp[index_PID_par] << endl;
					par_test_run = false;
					RestartSimulator(webSocket);
				}
				else {
					if (err_for_twiddle < best_err) {
						cout << "current error " << err_for_twiddle << " better than previous best err " << best_err << endl;
						best_err = err_for_twiddle;
						dp[index_PID_par] *= 1.1;
						cout << "tunning dp[" << index_PID_par<<"] by *1.1 to " << dp[index_PID_par]<< endl;
					
						index_PID_par++;
						//reset flag
						par_test_run = true;
						flag_after_worse_run = false;
						RestartSimulator(webSocket);
					}
					else {
						if (flag_after_worse_run) {
							cout << "current error " << err_for_twiddle << " is still not better than previous best err " << best_err << endl;
							p[index_PID_par] += dp[index_PID_par];
							cout << "tunning p[" << index_PID_par << "] by +1*dp to " << p[index_PID_par] << endl;
							dp[index_PID_par] *= 0.9;
							cout << "tunning d[" << index_PID_par << "] by *0.9 to " << dp[index_PID_par] << endl;
							
							index_PID_par++;
							//reset flag
							par_test_run = true;
							flag_after_worse_run = false;
							
							RestartSimulator(webSocket);

						}
						else {
							cout << "current error " << err_for_twiddle <<" is not better than previous best err " << best_err << endl;
							p[index_PID_par] -= 2 * dp[index_PID_par];
							cout << "tunning p[" << index_PID_par << "] by -2*dp to " << p[index_PID_par] << endl;
							flag_after_worse_run = true;
							RestartSimulator(webSocket);
						}
					}
				}
			}
			else {
				cout << "\n\n====================BEST PARAMETERS=============" << endl;
				cout << "After Iter:" << iter << endl;
				cout << "Kp = " << p[0] << endl;
				cout << "Ki = " << p[1] << endl;
				cout << "Kd = " << p[2] << endl;
				stop_twiddle = true;
				ofstream  twiddle_result;
				twiddle_result.open("twiddle_result.txt");
				twiddle_result << "Kp = " << p[0] << "\n";
				twiddle_result << "Ki = " << p[1] << "\n";
				twiddle_result << "Kd = " << p[2] <<"\n";
				twiddle_result <<"After Iter:" << iter << "\n";
				twiddle_result.close();
				


			}
		}
		
		index_collection++;
		/*cout << "index_collection:" << index_collection << endl;*/
	}
	
}
//void PID::SetPIDPar(double twiddle_p[3]) {
//	Kp = twiddle_p[0];
//	Ki = twiddle_p[1];
//	Kd = twiddle_p[2];
//
//}