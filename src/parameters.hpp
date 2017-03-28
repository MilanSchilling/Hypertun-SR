#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

struct parameters {
	// Occupancy grid size used for re-sampling
	int sz_occ;

	// Number of iterations the algorithm is allowed to run
	int n_iters;

	// Lower and upper threshold for validating disparities
	double t_lo;
	double t_hi;

    // image height and width
    int H;
    int W;
};

#endif // PARAMETERS_HPP