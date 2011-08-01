/*
 * solver_node.cpp
 *
 *  Created: April 2011
 *  Author: Chris Burbridge
 *
 *  A ROS node that provides a Rubik's cube solving
 *  service. The service is a custom type rubiks_solver/SolveService,
 *  advertised as '/rubiks_solve'.
 *
 *  The cube is solved by the Kociemba method implemented by 
 *  Greg Schmidt. The original code license is not specified, but open 
 *  and "free to redistribute". This code is GPL.
 *  The solving implementation is inside solvelib directory, with only
 *  minor alterations to make it compile in Linux, incorporate a
 *  timeout, and exploit ROS out.
 *  
 *  The request is as follows:
 *     string cubeState
 *     int32 timeout
 *
 *  cubeState is a string specifying the colour configureation
 *  in the same format as solve_cmd expects:
 *       "FaceSpec1 FaceSpec2 FaceSpec3 FaceSpec4 FaceSpec5 FaceSpec6"
 *  
 *     where each face specifier is of the form f:mmmmmmmmm
 *    
 *     and f (face) must be one of {U,D,F,B,L,R} and m (marker)
 *     can be any printable ASCII character.  Grasp the left or
 *     right side of the cube with one hand and rotate it so that
 *     the face under consideration is in front of you.  The order
 *     of the markers is left-to-right, top-to-bottom with respect
 *     to each face.  For example, consider the following "unfolded"
 *     cube:
 *                  Red Red Wht
 *                  Yel Wht Wht
 *                  Wht Red Red
 *  
 *     Wht Wht Grn  Org Yel Yel  Blu Red Org  Yel Grn Grn
 *     Red Org Org  Yel Yel Yel  Blu Red Org  Blu Grn Blu
 *     Yel Wht Grn  Org Wht Org  Yel Blu Blu  Red Grn Red
 *  
 *                  Blu Org Blu
 *                  Grn Blu Grn
 *                  Wht Org Grn
 *  
 *     With the faces above corresponding to:
 *    
 *          Up
 *     Left Front Right Back
 *          Down
 *    
 *     Then one possible string to describe the state is:
 *    
 *     "L:WWGROOYWG R:BROBROYBB U:RRWYWWWRR D:BOBGBGWOG	F:OYYYYYOWO B:YGGBGBRGR"
 *    
 *     Note: that the choice of marker characters is completely
 *     arbitrary, one must only use them consistently within
 *     any given cube configuration specification.  Also, the
 *     order of each face specification is arbitrary.
 *
 * timeout is an integer specifying how many seconds maximum to spend searching
 * for a good solution.
 * 
 * The reply is as follows:
 *   bool success
 *   string status
 *   string[] solution
 *
 * success: true if a solution was found, false if not - e.g bad spec
 * status:  string saying outcome
 * solution[]: an array of strings giving the solution.
 *             the solution strings tell which face to turn, using standard notation
 *
 */

#include "ros/ros.h"
#include "rubiks_solver/SolveService.h"
#include "solvelib/solver.h"     // Solver
#include "solvelib/kocicube.h"	// KociembaCube
#include "solvelib/facecube.h"	// FaceletCube
#include "solvelib/cubepars.h"	// CubeParser

bool solve(rubiks_solver::SolveService::Request &req,
		rubiks_solver::SolveService::Response &res) {
	ROS_INFO("Rubik's Solver: request: state=%s, timeout=%d", req.cubeState.c_str(), req.timeout);

	unsigned int status;
	FaceletCube faceletCube;
	CubeParser cubeParser;
	std::vector<std::string> solution;
	KociembaCube cube;
	std::string returnStatus("");

	// Parse the input and initialize a "FaceletCube".
	// The FaceletCube represents the cube by the markings
	//   of the 54 individual facelets as given in req.cubeState
	if ((status = cubeParser.ParseInputROS(req.cubeState, faceletCube)) != CubeParser::VALID)
	{
		returnStatus =  std::string(cubeParser.ErrorText(status));
		res.success = false;
	} else {
		// Validate the facelet representation in terms	of
		//   legal cubie markings, permutation, and parity
		//   and initialize a "standard" cube.  The standard
		//   cube represents the cube state in terms of cubie
		//   permutation and parity.
		if ((status = faceletCube.Validate(cube)) != FaceletCube::VALID)
		{
			returnStatus =  std::string(faceletCube.ErrorText(status));
			res.success = false;
		} else {
			// Create a solver, initialize the move mapping and
			//   pruning tables, and invoke the search for a
			//   solution.  Since the cube is in a valid configuration
			//   at this point, a solution should always be found.
			Solver solver(req.timeout);
			solver.InitializeTables();
			int solveStatus = solver.Solve(cube);
			switch(solveStatus) {
				case (Solver::ABORT):
					returnStatus = std::string("Solution hunt aborted.");
					break;
				case (Solver::FOUND):
					returnStatus = std::string("Solution found.");
					break;
				case (Solver::NOT_FOUND):
					returnStatus = std::string("Solution not found.");
					break;
				case (Solver::OPTIMUM_FOUND):
					returnStatus = std::string("Optimal solution found.");
					break;
				case (Solver::TIMEOUT):
					returnStatus = std::string("Solution hunt timed out; sub-optimal solution?.");
					break;
			}

			solution = solver.GetSolution();
			if (solution.size() < 1) {
				res.success = false;
			} else {
				res.success = true;
			}
		}
	}



	res.solution = solution;
	res.status = returnStatus;

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rubiks_solver");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("/rubiks_solve", solve);
	ROS_INFO("Rubik's solving service ready [/rubiks_solve [rubiks_solver/SolveService]]....");
	ros::spin();

	return 0;
}
