/* ============================================================================
 * D Y N I B E X - STL formula verification on tube
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
#ifndef CSTL_H
#define CSTL_H

#include <iostream>
#include <vector>
#include "ibex.h"

using namespace std;
using namespace ibex;

typedef pair<int, pair<double, double>> Satisf_Signal;

// Function prototypes
vector<Satisf_Signal> neg_stl(const vector<Satisf_Signal>& sp);
vector<Satisf_Signal> and_stl(const vector<Satisf_Signal>& sp1, const vector<Satisf_Signal>& sp2);
vector<Satisf_Signal> or_stl(const vector<Satisf_Signal>& sp1, const vector<Satisf_Signal>& sp2);
void print_Satisf_Signals(const vector<Satisf_Signal>& Satisf_Signals);
vector<Satisf_Signal> until_stl(const vector<Satisf_Signal>& list1, const vector<Satisf_Signal>& list2, pair<double, double> time_itv);
vector<Satisf_Signal> Finally(const vector<Satisf_Signal>& list1, pair<double, double> time_itv);
vector<Satisf_Signal> Globally(const vector<Satisf_Signal>& list1, pair<double, double> time_itv);
std::vector<vector<Satisf_Signal>> predicate_satisfaction(ibex::simulation& sim, const vector<IntervalVector>& predicate_list);
std::vector<vector<Satisf_Signal>> predicate_satisfaction_jn(const std::vector<std::pair<IntervalVector, Interval>>& jn_box, const vector<IntervalVector>& predicate_list);

//predicate handling
std::vector<vector<Satisf_Signal>> predicate_satisfaction_jn(const std::vector<std::pair<IntervalVector, Interval>>& jn_box, const vector<IntervalVector>& predicate_list);
std::vector<std::pair<IntervalVector, Interval>> Append_tube(const std::vector<std::pair<IntervalVector, Interval>>& old_tube, const std::vector<std::pair<IntervalVector, Interval>>& extension_tube);
std::vector<std::pair<IntervalVector, Interval>> Sim_to_jn_tube(ibex::simulation& sim);
#endif // CSTL_H
