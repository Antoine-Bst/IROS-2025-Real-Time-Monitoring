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

#include <iostream>
#include <vector>
#include "ibex.h"

using namespace std;
using namespace ibex;

typedef pair<int, pair<double, double>> Satisf_Signal;

int and_unitary(int val1, int val2) {
    return (val1 > 0 && val2 > 0) ? max(val1, val2) : 0;
}

int or_unitary(int val1, int val2) {
    if (val1 > 0 && val2 > 0) return min(val1, val2);
    if (val1 == 0 && val2 > 0) return val2;
    if (val2 == 0 && val1 > 0) return val1;
    return 0;
}

vector<Satisf_Signal> list_compactor(const vector<int>& signal) {
    vector<Satisf_Signal> Satisf_Signals;
    int valeur_courante = signal[0];
    int debut = 0;

    for (size_t i = 1; i < signal.size(); ++i) {
        if (signal[i] != valeur_courante) {
            Satisf_Signals.push_back(make_pair(valeur_courante, make_pair(debut, i)));
            debut = i;
            valeur_courante = signal[i];
        }
    }
    Satisf_Signals.push_back(make_pair(valeur_courante, make_pair(debut, signal.size())));
    return Satisf_Signals;
}

vector<Satisf_Signal> list_compactor_until(const vector<int>& signal, double step_time) {
    vector<Satisf_Signal> Satisf_Signals;
    int valeur_courante = signal[0];
    double debut = 0;
    Satisf_Signals.push_back(make_pair(valeur_courante, make_pair(debut, signal.size() * step_time)));
    return Satisf_Signals;
}

vector<int> decompactor(const vector<Satisf_Signal>& Satisf_Signals) {
    double longueur_totale = 0;
    for (const auto& Satisf_Signal : Satisf_Signals)
        longueur_totale = max(longueur_totale, Satisf_Signal.second.second);

    vector<int> signal(static_cast<int>(longueur_totale), 2);
    for (const auto& Satisf_Signal : Satisf_Signals) {
        for (int i = Satisf_Signal.second.first; i < Satisf_Signal.second.second; ++i)
            signal[i] = Satisf_Signal.first;
    }
    return signal;
}

vector<Satisf_Signal> completator(const vector<Satisf_Signal>& sp, double fin_max) {
    if (sp.empty()) return {make_pair(2, make_pair(0, fin_max))};
    vector<Satisf_Signal> resultat;
    double dernier_fin = 0;
    for (const auto& Satisf_Signal : sp) {
        if (Satisf_Signal.second.first > dernier_fin)
            resultat.push_back(make_pair(2, make_pair(dernier_fin, Satisf_Signal.second.first)));
        resultat.push_back(Satisf_Signal);
        dernier_fin = Satisf_Signal.second.second;
    }
    if (dernier_fin < fin_max)
        resultat.push_back(make_pair(2, make_pair(dernier_fin, fin_max)));
    return resultat;
}

vector<Satisf_Signal> merge_Satisf_Signals(const vector<Satisf_Signal>& Satisf_Signals) {
    if (Satisf_Signals.empty()) {std::cout<<"Err : Signal empty"<<std::endl; return {};}

    vector<Satisf_Signal> merged;
    merged.push_back(Satisf_Signals[0]);

    for (size_t i = 1; i < Satisf_Signals.size(); ++i) {
        if (Satisf_Signals[i].first == merged.back().first) {
            merged.back().second.second = Satisf_Signals[i].second.second;
        } else {
            merged.push_back(Satisf_Signals[i]);
        }
    }
    return merged;
}

vector<Satisf_Signal> merge_Satisf_Signals_Mink(const vector<Satisf_Signal>& Satisf_Signals) {
    if (Satisf_Signals.empty()) {std::cout<<"Err : Signal empty"<<std::endl; return {};}

    vector<Satisf_Signal> merged;
    merged.push_back(Satisf_Signals[0]);

    for (size_t i = 1; i < Satisf_Signals.size(); ++i) {
        if (Satisf_Signals[i].first>0 && merged.back().first>0) {
            merged.back().second.second = Satisf_Signals[i].second.second;
            merged.back().first = 2;
        } else {
            merged.push_back(Satisf_Signals[i]);
        }
    }
    return merged;
}


vector<Satisf_Signal> neg_stl(const vector<Satisf_Signal>& sp) {
    vector<Satisf_Signal> negated_Satisf_Signals;
    for (const auto& Satisf_Signal : sp) {
        int negated_value = (Satisf_Signal.first == 0) ? 1 : (Satisf_Signal.first == 1 ? 0 : Satisf_Signal.first);
        negated_Satisf_Signals.push_back(make_pair(negated_value, Satisf_Signal.second));
    }
    return negated_Satisf_Signals;
}

vector<Satisf_Signal> and_stl(const vector<Satisf_Signal>& sp1, const vector<Satisf_Signal>& sp2) {
    double fin_max = max(sp1.empty() ? 0 : sp1.back().second.second, sp2.empty() ? 0 : sp2.back().second.second);
    vector<Satisf_Signal> sp3 = completator(sp1, fin_max);
    vector<Satisf_Signal> sp4 = completator(sp2, fin_max);
    vector<Satisf_Signal> resultat;
    size_t i = 0, j = 0;

    while (i < sp3.size() && j < sp4.size()) {
        auto v1 = sp3[i].first;
        auto v2 = sp4[j].first;
        double debut_inter = max(sp3[i].second.first, sp4[j].second.first);
        double fin_inter = min(sp3[i].second.second, sp4[j].second.second);
        
        if (debut_inter < fin_inter) {
            resultat.push_back(make_pair(and_unitary(v1, v2), make_pair(debut_inter, fin_inter)));
        }
        
        if (sp3[i].second.second <= sp4[j].second.second) ++i;
        else ++j;
    }
    return merge_Satisf_Signals(resultat);
}


vector<Satisf_Signal> or_stl(const vector<Satisf_Signal>& sp1, const vector<Satisf_Signal>& sp2) {
    double fin_max = max(sp1.empty() ? 0 : sp1.back().second.second, sp2.empty() ? 0 : sp2.back().second.second);
    vector<Satisf_Signal> sp3 = completator(sp1, fin_max);
    vector<Satisf_Signal> sp4 = completator(sp2, fin_max);
    vector<Satisf_Signal> resultat;
    size_t i = 0, j = 0;

    while (i < sp3.size() && j < sp4.size()) {
        auto v1 = sp3[i].first;
        auto v2 = sp4[j].first;
        double debut_inter = max(sp3[i].second.first, sp4[j].second.first);
        double fin_inter = min(sp3[i].second.second, sp4[j].second.second);
        
        if (debut_inter < fin_inter) {
            resultat.push_back(make_pair(or_unitary(v1, v2), make_pair(debut_inter, fin_inter)));
        }
        
        if (sp3[i].second.second <= sp4[j].second.second) ++i;
        else ++j;
    }
    return merge_Satisf_Signals(resultat);
}


vector<Satisf_Signal> completator_until(const vector<Satisf_Signal>& sp, double fin_max) {
    if (sp.empty()) return {make_pair(3, make_pair(0, fin_max))};
    vector<Satisf_Signal> resultat;
    double dernier_fin = 0;
    for (const auto& Satisf_Signal : sp) {
        if (Satisf_Signal.second.first > dernier_fin)
            resultat.push_back(make_pair(3, make_pair(dernier_fin, Satisf_Signal.second.first)));
        resultat.push_back(Satisf_Signal);
        dernier_fin = Satisf_Signal.second.second;
    }
    if (dernier_fin < fin_max)
        resultat.push_back(make_pair(3, make_pair(dernier_fin, fin_max)));
    return resultat;
}

vector<Satisf_Signal> offset_time(const vector<Satisf_Signal>& intervals, double offset) {
    vector<Satisf_Signal> offset_intervals;
    for (const auto& interval : intervals) {
        offset_intervals.push_back(make_pair(interval.first, make_pair(interval.second.first + offset, interval.second.second + offset)));
    }
    return offset_intervals;
}

vector<Satisf_Signal> merge_stl_titv(const vector<Satisf_Signal>& sp1, const vector<Satisf_Signal>& sp2) {
    double fin_max = max(sp1.empty() ? 0 : sp1.back().second.second, sp2.empty() ? 0 : sp2.back().second.second);
    vector<Satisf_Signal> sp3 = completator(sp1, fin_max);
    vector<Satisf_Signal> sp4 = completator(sp2, fin_max);

    vector<Satisf_Signal> merged_result;
    size_t i = 0, j = 0;

    while (i < sp3.size() && j < sp4.size()) {
        int v1 = sp3[i].first;
        int v2 = sp4[j].first;
        double d1 = sp3[i].second.first, f1 = sp3[i].second.second;
        double d2 = sp4[j].second.first, f2 = sp4[j].second.second;

        double start_inter = max(d1, d2);
        double end_inter = min(f1, f2);

        if (start_inter < end_inter) {
            int merged_value;
            if (v1 == 2) {
                merged_value = v2;
            } else if (v2 == 2) {
                merged_value = v1;
            } else {
                merged_value = v1;
            }
            merged_result.push_back(make_pair(merged_value, make_pair(start_inter, end_inter)));
        }

        if (f1 <= f2) {
            i++;
        } else {
            j++;
        }
    }
    return merge_Satisf_Signals(merged_result);
}

void print_Satisf_Signals(const vector<Satisf_Signal>& Satisf_Signals) {
    cout<<"(Satisfaction value"<< ", [" << "time interval"<< "[)\n";
    for (const auto& Satisf_Signal : Satisf_Signals) {
        if (Satisf_Signal.first != 2)
        {
            cout<<"("<<Satisf_Signal.first << ", [" << Satisf_Signal.second.first << ", " << Satisf_Signal.second.second << "[)\n";
        }
        else
        {
            cout<<"("<< "[0,1]" << ", [" << Satisf_Signal.second.first << ", " << Satisf_Signal.second.second << "[)\n";
        }
        
    }
}

Satisf_Signal computeIntersection(const Satisf_Signal& elem1, const Satisf_Signal& elem2, const int& value) {
    Satisf_Signal result;
            double start = std::max(elem1.second.first, elem2.second.first);
            double end = std::min(elem1.second.second, elem2.second.second);
            
            if (0 <= start && start < end) { // Valid intersection
                result = {value, {start, end}};
            }
            else if (0<end && start < end)
            {
                result = {value, {0, end}};
            }
            else
            {
                result = {-1, {-1, 0}};
            }
        return result;
}


vector<Satisf_Signal> until_stl(const vector<Satisf_Signal>& list1, const vector<Satisf_Signal>& list2, pair<double, double> time_itv) {
const double fin_max = max(list1.back().second.second, list2.back().second.second);
    vector<Satisf_Signal> sp1 = completator(merge_Satisf_Signals(list1), fin_max);
    vector<Satisf_Signal> sp2 = completator(merge_Satisf_Signals(list2), fin_max);

double maxtime = max(sp2.back().second.second - time_itv.second, sp1.back().second.second - time_itv.second);
   std::vector<Satisf_Signal> result = {{0, {0,maxtime}}};
   std::vector<Satisf_Signal> result_final = {{0, {0,maxtime}}};
   Satisf_Signal intersection;
   std::vector<Satisf_Signal> temp;
   std::vector<Satisf_Signal> temp_final;
   std::vector<Satisf_Signal> merged_sp1= merge_Satisf_Signals_Mink(sp1);
    
    for (size_t i = 0; i < merged_sp1.size(); i++)
    {
        for (size_t j = 0; j < sp2.size(); j++)
        {
            if (merged_sp1[i].first==1 && sp2[j].first==1){
                Satisf_Signal intersection = computeIntersection(merged_sp1[i], sp2[j], 1);
                intersection.second.first = intersection.second.first-time_itv.second;
                intersection.second.second = intersection.second.second-time_itv.first;
                intersection = computeIntersection(merged_sp1[i], intersection, 1);
                
                if (intersection.first != -1)
                {
                temp = {{0,{0,intersection.second.first}}, intersection, {0, {intersection.second.second,maxtime}}};
                result = or_stl(result, temp);
                }            
            }
            else if (merged_sp1[i].first>0 && sp2[j].first>0)
            {   
                Satisf_Signal intersection = computeIntersection(merged_sp1[i], sp2[j], 2);
                intersection.second.first = intersection.second.first-time_itv.second;
                intersection.second.second = intersection.second.second-time_itv.first;
                intersection = computeIntersection(merged_sp1[i], intersection, 2);
                
                if (intersection.first != -1)
                {   
                temp = {{0,{0,intersection.second.first}}, intersection, {0, {intersection.second.second,maxtime}}};
                result = or_stl(result, temp);
                }     
            }
        }
    }
    for (size_t i = 0; i < sp1.size(); i++)
    {
        for (size_t j = 0; j < sp2.size(); j++)
        {
            if (sp1[i].first==1 && sp2[j].first==1){
                Satisf_Signal intersection = computeIntersection(sp1[i], sp2[j], 1);
                
                intersection.second.first = intersection.second.first-time_itv.second;
                intersection.second.second = intersection.second.second-time_itv.first;
                intersection = computeIntersection(sp1[i], intersection, 1);
                if (intersection.first != -1)
                {
                temp_final = {{0,{0,intersection.second.first}}, intersection, {0, {intersection.second.second,maxtime}}};
                result_final = or_stl(result_final, temp_final);
                }            
            }
        }
    }

    result = or_stl(result_final, result); 
    return result;
}

vector<Satisf_Signal> Finally(const vector<Satisf_Signal>& list1, pair<double, double> time_itv){

    vector<Satisf_Signal> temp = {{1,{0, list1.back().second.second + time_itv.second}}};
    return until_stl(temp, list1, time_itv);
}
vector<Satisf_Signal> Globally(const vector<Satisf_Signal>& list1, pair<double, double> time_itv){

    return neg_stl(Finally(neg_stl(list1),time_itv));
}

std::vector<vector<Satisf_Signal>> predicate_satisfaction(ibex::simulation& sim, const vector<IntervalVector>& predicate_list) {
    std::vector<std::pair<IntervalVector, Interval>> jn_box;
    for (const auto& sol : sim.list_solution_g) {
        if (sol.box_jn && (sol.time_j.lb()>=0)) { // Ensure it's not NULL
            jn_box.push_back({*sol.box_j1,sol.time_j});
        }
    }
    //jn_box;
    std::vector<vector<Satisf_Signal>> result;
    for (size_t j = 0; j < predicate_list.size(); j++)
    {
      std::vector<Satisf_Signal> P_satisf;
        for (size_t i = 0; i < jn_box.size(); i++)
        {
          int s_val = 0;
          if (jn_box[i].first.is_subset(predicate_list[j]))
          {
            s_val = 1;
          }
          else if (jn_box[i].first.intersects(predicate_list[j]))
          {
            s_val = 2;
          }
          
          if (s_val != 2)
          {
            P_satisf.push_back({s_val,{jn_box[i].second.lb(), jn_box[i].second.ub()}});
          }
          else
          {
            P_satisf.push_back({s_val,{jn_box[i].second.lb(), jn_box[i].second.ub()}});
          }
      }
      result.push_back(P_satisf);
    }

  return result;
}
