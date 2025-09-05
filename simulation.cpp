/* ============================================================================
 * D Y N I B E X - STL example
 * ============================================================================
 * Copyright   : ENSTA
 * License     : This program can be distributed under the terms of the GNU LGPL.
 *               See the file COPYING.LESSER.
 *
 * Author(s)   : Antoine Besset, Joris Tillet and Julien Alexandre dit Sandretto
 * Created     : Jul 22, 2025
 * Modified    : Jul 22, 2025
 * Sponsored   : This research benefited from the support of the "STARTS Projects - CIEDS - Institut Polytechnique"
 * ---------------------------------------------------------------------------- */

#include "ibex.h"
#include <iostream>
#include "CSTL.h"

using namespace ibex;
using namespace std;


int main(){
  double mean = 2;
  double variance = 0.1;

  double meanP = 0;
  double varianceP =  0.1;

  double meanD = 0;
  double varianceD = 0.1;

  const int n= 6; ///number of state of the system
  Variable y(n);
  IntervalVector Box(n);
  IntervalVector yinit(n);
  yinit[0] = Interval(1);  //x
  yinit[1] = Interval(1);   //x'
  yinit[2] = Interval(1);  //x
  yinit[3] = Interval(1);   //x'
  yinit[4] = Interval(0);
  yinit[5] = Interval(0);

  Interval K = Interval(mean - variance, mean + variance);
  Interval P = Interval(meanP - varianceP, meanP + varianceP);
  Interval D = Interval(meanD - varianceD, meanD + varianceD); //Interval Parameters
  double tsimu=10.0;
  //---------------simulation--------------------
  Function ydot = Function(y,Return(K*(y[2]-y[0]) + P,
  K*(y[3]-y[1]) + P,
  1.5*y[2]-0.5*y[2]*y[3], 
  -0.2*y[3]+0.2*y[2]*y[3],
  K*(y[5]-y[4]) + D,
  0.2*y[2]-0.2*1.4
  ));
  ivp_ode problem = ivp_ode(ydot,0.0,yinit);    
  simulation simu = simulation(&problem,tsimu,RK4,1e-6);

  simu.run_simulation();
  ///-----------------------end of the simulation-------------------------

      vector<IntervalVector> p_list; //predicate list

      IntervalVector predicate1(n); //v
        predicate1[0] = Interval(5,5.75);
        predicate1[1] = Interval(2,3);
        predicate1[2] = Interval(-1000,1000);
        predicate1[3] = Interval(-1000,1000);
        predicate1[4] = Interval(0.5,1);
        predicate1[5] = Interval(-1000,1000);


       IntervalVector predicate2(n); //w
        predicate2[0] = Interval(0.75,1.5);
        predicate2[1] = Interval(6,6.75);
        predicate2[2] = Interval(-1000,1000);
        predicate2[3] = Interval(-1000,1000);
        predicate2[4] = Interval(1.3,1.75);
        predicate2[5] = Interval(-1000,1000);


        IntervalVector predicate3(n); //p
        predicate3[0] = Interval(1,3);
        predicate3[1] = Interval(2,5);
        predicate3[2] = Interval(-1000,1000);
        predicate3[3] = Interval(-1000,1000);
        predicate3[4] = Interval(0,1.2);
        predicate3[5] = Interval(-1000,1000);


        IntervalVector predicate4(n); //q
        predicate4[0] = Interval(-0.5,0.5);
        predicate4[1] = Interval(2,4);
        predicate4[2] = Interval(-1000,1000);
        predicate4[3] = Interval(-1000,1000);
        predicate4[4] = Interval(0,1);
        predicate4[5] = Interval(-1000,1000);

        p_list = {predicate1,predicate2, predicate3, predicate4}; //list of predicate set (Interval Vectors/Hyper-boxes)



      ////------------------------STL Checking--------------------------------
        vector<Satisf_Signal> Phi1; //declaration of subformulas satisfaction signals
        vector<Satisf_Signal> Phi2;
        vector<Satisf_Signal> Phi3;
        vector<Satisf_Signal> Phi4;
        vector<Satisf_Signal> Phi5;

        vector<Satisf_Signal> V_p; //predicates satisfaction signals
        vector<Satisf_Signal> W_p;
        vector<Satisf_Signal> P_p;
        vector<Satisf_Signal> Q_p;
        
        std::vector<vector<Satisf_Signal>> P_Satisfaction_signals = predicate_satisfaction(simu, p_list); //evaluation on the simulated tube

        V_p=P_Satisfaction_signals[0]; //assigning every predicate to its satisfaction signals
        W_p=P_Satisfaction_signals[1];
        P_p=P_Satisfaction_signals[2];
        Q_p=P_Satisfaction_signals[3];

        Phi1 = or_stl(neg_stl(V_p), Finally(W_p, {1.6, 3.4})); //building the formula tree
        Phi4 = Globally(Phi1, {0,4.5});
        Phi2 = neg_stl(P_p);
        Phi3 = Q_p;
        Phi5 = and_stl(until_stl(Phi2,Phi3,{8,9})  , Phi4);

        print_Satisf_Signals(Phi5); //print the satisfaction signal of a formula
        
return 1;
}
