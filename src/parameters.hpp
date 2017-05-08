#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

struct parameters {
	// Occupancy grid size used for re-sampling
	int sz_occ;

	// Number of iterations the algorithm is allowed to run
	int n_iters;

	// Lower and upper threshold for validating disparities
	float t_lo;
	float t_hi;

    // image height and width
    int H;
    int W;

	// grid height and width
	int H_bar;
	int W_bar;

	// threshold for image gradient
	int im_grad;
};

#endif // PARAMETERS_HPP