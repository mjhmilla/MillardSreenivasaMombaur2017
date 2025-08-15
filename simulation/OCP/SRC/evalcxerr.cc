#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <sstream>

#include <boost/numeric/odeint.hpp>
#include <boost/tokenizer.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/Dynamics.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

#include "wholebodylifter2d_Enums.h"
#include "wholebodylifter2d_EnumMap.h"
#include "wholebodylifter2d_Model.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace boost;

using namespace std;
using namespace std::placeholders;

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool is_double(string const & s) {
		try {
			stod(s);
		} catch(...) {
			return false;
		}
		return true;
}

template <typename M> M csv2Mat(string const & fn) {
	ifstream ifs(fn);
	if (!ifs.is_open()) {
		return M();
	}

	typedef tokenizer< escaped_list_separator<char> > Tokenizer;

	vector< string > vec;
	string line;
	vector< vector< string > > rows;

	while (getline(ifs, line)) {
		Tokenizer tok(line);
		vec.assign(tok.begin(),tok.end());
		rows.push_back(vec);
	}
	ifs.close();

	int row_count = rows.size();
	int col_count = row_count > 0 ? rows[0].size() : 0;

	int offset = is_double(rows.front().back()) ? 0 : 1;

	vector<double> values;
	for(int i = 0; i < row_count; i++) {
		int col_count_i = rows[i].size();
		if(col_count_i == col_count) {
			transform(rows[i].begin(), rows[i].end() - offset, back_inserter(values),
								[](string const & s) -> double { return stod(s); });
		}
	}

	return Eigen::Map<const Eigen::Matrix<
			typename M::Scalar,
			M::RowsAtCompileTime,
			M::ColsAtCompileTime,
			Eigen::RowMajor>>(values.data(), row_count, values.size() / row_count);
}

WholeBodyLifterModel model;

int main (int argc, char *argv[])
{
	rbdl_check_api_version (RBDL_API_VERSION);

	string model_file = "../DAT/matt2d.lua";
	string datfile_name = "../DAT/wholebodylifter2d.dat";

	model.loadFromFile (model_file, datfile_name, false);
	model.loadPoints (model_file, false);
	model.loadConstraintSets (model_file, false);

	MatrixXd m = csv2Mat<MatrixXd>(string("RES/wholebodylifter2d.csv"));

	for (int i = 0; i < m.rows(); i++) {
		double t = m(i, 0);
		VectorNd q = m.row(i).segment(1, m.cols() - 1).transpose();

		if (i <= m.rows() / 2) {
			ConstraintSet cs = model.constraint_sets[CSBothFeetFlat2D];
			VectorNd e(cs.size());
			CalcConstraintsPositionError(model.model, q, cs, e);
			int kk;
			double vv = e.cwiseAbs().maxCoeff(&kk);
			cout << setw(8) << setprecision(5) << t << " , " << setw(10) << e.norm() << " , " << setw(10) << vv << " (" << kk << ")" << endl;
		} else {
			ConstraintSet cs = model.constraint_sets[CSBothFeetFlat2DBoxAttach];
			VectorNd e(cs.size());
			CalcConstraintsPositionError(model.model, q, cs, e);
			int kk;
			double vv = e.cwiseAbs().maxCoeff(&kk);
			cout << setw(8) << setprecision(5) << t << " , " << setw(10) << e.norm() << " , " << setw(10) << vv << " (" << kk << ")" << endl;
		}
	}

	return 0;
}
