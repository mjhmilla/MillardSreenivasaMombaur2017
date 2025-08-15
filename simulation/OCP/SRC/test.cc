#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

#include <boost/numeric/odeint.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/Dynamics.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

#include "wholebodylifter2d_Enums.h"
#include "wholebodylifter2d_EnumMap.h"
#include "wholebodylifter2d_Model.h"

using namespace std;
using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;

#define NQ    PosNameLast
#define NU    ControlNameLast
#define NX    StateNameLast

// --- Helpers 4 INT -------------------------------------------------------- //
typedef array< double, NX > state_t;
typedef vector< pair<double, state_t> > tx_traj_t;

struct state_observer {
  tx_traj_t & tx_traj;

  state_observer( tx_traj_t & tx_traj ) : tx_traj( tx_traj)  { }

  void operator()( const state_t &x , double t ) {
    tx_traj.push_back(make_pair(t, x));
  }
};
// -------------------------------------------------------------------------- //

// --- Helpers 4 CSV -------------------------------------------------------- //
ostream & operator<< (ostream &out, pair<double, state_t> const &tx);
// -------------------------------------------------------------------------- //

void rhs(const state_t & x , state_t & dxdt , const double t);

WholeBodyLifterModel model;

array<double, 1> p = { 0 };
array<double, NU> u;

vector< pair<double, state_t> > tx_traj;

typedef runge_kutta_dopri5<state_t> rkdp5;

int main (int argc, char *argv[])
{
  rbdl_check_api_version (RBDL_API_VERSION);

  string model_file = "matt2d_box_1kg.lua";
  string datfile_name = "wholebodylifter2d.dat";

  model.loadFromFile (model_file, datfile_name, false);
  model.loadPoints (model_file, false);
  model.loadConstraintSets (model_file, false);

  u.fill(0.0);

	state_t x0 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               };

	ofstream ofs("out.csv", ios::out);

	integrate_adaptive(rkdp5(), rhs, x0, 0.0, 1.0, 0.001, state_observer(tx_traj));

	for_each (tx_traj.begin(), tx_traj.end(),
						[&ofs](pair<double, state_t> const & tx) { ofs << tx; });

	return 0;
}


void rhs(const state_t & x , state_t & dxdt , const double t) {
	model.updateState(x.data(), u.data(), p.data(), CSBothFeetFlat2DBoxAttach);
	model.calcForwardDynamicsRhs (dxdt.data());
}

// --- Helpers 4 CSV -------------------------------------------------------- //

ostream & operator<< (ostream &out, pair<double, state_t> const &tq) {
  out << tq.first << ", ";
  for (unsigned i = 0; i < NQ; i++) {
    out << tq.second[i] << ", ";
  }
  out << "\r\n";
  return out;
}

// -------------------------------------------------------------------------- //
