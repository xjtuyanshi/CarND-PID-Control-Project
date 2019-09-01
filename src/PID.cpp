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

double PID::TotalError(double cte) {
  /**
   *  Calculate and return the total error -for twiddle use
   */
	
	total_error += abs(cte);
	
	return total_error;
}
double PID::GetSteerValue() {
	/**
	 *  Calculate and return the steer value
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
	total_error = 0;
	index_collection = 0;
	cout << "current PID values before restart:" << endl;
	cout << p[0] << "," << p[1] << "," << p[2] << endl;

	std::string msg = "42[\"reset\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}
void PID::ResetParameters() {
	Init(p[0], p[1], p[2]);
	total_error = 0;
	index_collection = 0;
	cout << "current PID values:" << endl;
	cout << p[0] << "," << p[1] << "," << p[2] << endl;
}


void PID::SetWS(uWS::WebSocket<uWS::SERVER> ws) {
	webSocket = ws;


}
void PID::Twiddle() {
	// run the simulator for a while to collect errors
	if (!stop_twiddle) {
		cout << "Iter =" << iter << endl;
		//error collected -start to tune pid values
		cout << "sum of abs(cte) =" << total_error << endl;
		//double n_index = (index_collection -);
		total_error = total_error / index_collection;
		cout << "Normolized abs(cte) =" << total_error <<" after message steps:"<<index_collection<< endl;
		double sum_dp = dp[0] + dp[1] + dp[2];
		//reset index for PID parameters (0- P,1- I,2-D)
		if (index_PID_par == 3) {
			index_PID_par = 0;
			iter++;
		}
		if (sum_dp > 0.001 && iter < max_iter) {
			cout <<  "best error=" << best_err << endl;
			if (par_test_run) {
				p[index_PID_par] += dp[index_PID_par];
				cout<<"this iter:" <<iter<<",first run, tunning dp[" << index_PID_par << "] by +1* to " << dp[index_PID_par] << endl;
				par_test_run = false;
				//RestartSimulator(webSocket);
				//ResetParameters();
			}
			else {
				if (total_error < best_err) {
					cout << "current error " << total_error << " is better than previous best err " << best_err << endl;
					best_err = total_error;
					dp[index_PID_par] *= 1.1;
					cout << "tunning dp[" << index_PID_par<<"] by *1.1 to " << dp[index_PID_par]<< endl;
					
					index_PID_par++;
					//reset flag
					par_test_run = true;
					flag_after_worse_run = false;
					//RestartSimulator(webSocket);
					//ResetParameters();
				}
				else {
					if (flag_after_worse_run) {
						cout << "current error " << total_error << " is still not better than previous best err " << best_err << endl;
						p[index_PID_par] += dp[index_PID_par];
						cout << "tunning p[" << index_PID_par << "] by +1*dp to " << p[index_PID_par] << endl;
						dp[index_PID_par] *= 0.9;
						cout << "tunning d[" << index_PID_par << "] by *0.9 to " << dp[index_PID_par] << endl;
							
						index_PID_par++;
						//reset flag
						par_test_run = true;
						flag_after_worse_run = false;
						//ResetParameters();
						//RestartSimulator(webSocket);

					}
					else {
						cout << "current error " << total_error <<" is not better than previous best err " << best_err << endl;
						p[index_PID_par] -= 2 * dp[index_PID_par];
						cout << "tunning p[" << index_PID_par << "] by -2*dp to " << p[index_PID_par] << endl;
						flag_after_worse_run = true;
						//RestartSimulator(webSocket);
						//ResetParameters();
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
		/*cout << "index_collection:" << index_collection << endl;*/
	}
	
}
//void PID::SetPIDPar(double twiddle_p[3]) {
//	Kp = twiddle_p[0];
//	Ki = twiddle_p[1];
//	Kd = twiddle_p[2];
//
//}