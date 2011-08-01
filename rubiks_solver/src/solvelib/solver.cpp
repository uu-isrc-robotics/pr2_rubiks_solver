// solver.cpp - Cube Solver class implementation

#include "solver.h"		// Solver

#include <iostream>
#include <iomanip>
#include <ros/ros.h>



Solver::Solver(float timeout)
	// Phase 1 move mapping tables
	: twistMoveTable(cube),
	  flipMoveTable(cube),
	  choiceMoveTable(cube),
	// Phase 2 move mapping tables
	  cornerPermutationMoveTable(cube),
	  nonMiddleSliceEdgePermutationMoveTable(cube),
	  middleSliceEdgePermutationMoveTable(cube),
	
	// Phase 1 pruning tables
	  TwistAndFlipPruningTable(
		twistMoveTable, flipMoveTable,
		cube.Twist(), cube.Flip()),
	  TwistAndChoicePruningTable(
		twistMoveTable, choiceMoveTable,
		cube.Twist(), cube.Choice()),
	  FlipAndChoicePruningTable(
		flipMoveTable, choiceMoveTable,
		cube.Flip(), cube.Choice()),
	// Phase 2 pruning tables
	  CornerAndSlicePruningTable(
		cornerPermutationMoveTable, middleSliceEdgePermutationMoveTable,
		cube.CornerPermutation(), cube.MiddleSliceEdgePermutation()),
	  EdgeAndSlicePruningTable(
		nonMiddleSliceEdgePermutationMoveTable, middleSliceEdgePermutationMoveTable,
		cube.NonMiddleSliceEdgePermutation(), cube.MiddleSliceEdgePermutation())
{
	minSolutionLength = Huge;	// Any solution disovered will look better than this one!

	this->timeout = timeout;

	ROS_INFO("Solver initializing with timeout %f seconds",timeout);
}

Solver::~Solver()
{
}

void Solver::InitializeTables(void)
{
	// Phase 1 move mapping tables

	ROS_INFO("Initializing TwistMoveTable");
	twistMoveTable.Initialize((char*)"/tmp/Twist.mtb");

	ROS_INFO("Initializing FlipMoveTable");
	flipMoveTable.Initialize((char*)"/tmp/Flip.mtb");

	ROS_INFO("Initializing ChoiceMoveTable");
	choiceMoveTable.Initialize((char*)"/tmp/Choice.mtb");

	// Phase 2 move mapping tables

	ROS_INFO("Initializing CornerPermutationMoveTable");
	cornerPermutationMoveTable.Initialize((char*)"/tmp/CrnrPerm.mtb");

	ROS_INFO("Initializing NonMiddleSliceEdgePermutationMoveTable");
	nonMiddleSliceEdgePermutationMoveTable.Initialize((char*)"/tmp/EdgePerm.mtb");

	ROS_INFO("Initializing MiddleSliceEdgePermutationMoveTable");
	middleSliceEdgePermutationMoveTable.Initialize((char*)"/tmp/SlicPerm.mtb");

	// Phase 1 pruning tables
	
	ROS_INFO("Initializing TwistAndFlipPruningTable");
	TwistAndFlipPruningTable.Initialize((char*)"/tmp/TwstFlip.ptb");
	
	ROS_INFO("Initializing TwistAndChoicePruningTable");
	TwistAndChoicePruningTable.Initialize((char*)"/tmp/TwstChce.ptb");

	ROS_INFO("Initializing FlipAndChoicePruningTable");
	FlipAndChoicePruningTable.Initialize((char*)"/tmp/FlipChce.ptb");

	// Phase 2 pruning tables

	// Obviously a CornerAndEdgePruningTable doesn't make sense as it's size
	//   would be extremely large (i.e. 8!*8!)

	ROS_INFO("Initializing CornerAndSlicePruningTable");
	CornerAndSlicePruningTable.Initialize((char*)"/tmp/CrnrSlic.ptb");
	
	ROS_INFO("Initializing EdgeAndSlicePruningTable");
	EdgeAndSlicePruningTable.Initialize((char*)"/tmp/EdgeSlic.ptb");
}

int Solver::Solve(KociembaCube& scrambledCube)
{
	elapsed_time.start();
	solution.clear();
	int iteration = 1;
	int result = NOT_FOUND;

	// Make a copy of the scrambled cube for use later on
	cube = scrambledCube;

	// Establish initial cost estimate to goal state
	threshold1 = Phase1Cost(cube.Twist(), cube.Flip(), cube.Choice());

	nodes1 = 1;		// Count root node here
	solutionLength1 = 0;

	do
	{
		if (elapsed_time.check() > timeout){
			return TIMEOUT;
		}

		newThreshold1 = Huge;	// Any cost will be less than this

		// Perform the phase 1 recursive IDA* search
		result = Search1(cube.Twist(), cube.Flip(), cube.Choice(), 0);

		// Establish a new threshold for a deeper search
		threshold1 = newThreshold1;
		
		// Count interative deepenings
		iteration++;
	} while (result == NOT_FOUND);
	ROS_INFO("Solve complete.");
	return result;
}

int Solver::Search1(int twist, int flip, int choice, int depth)
{
	int cost, totalCost;
	int move;
	int power;
	int twist2, flip2, choice2;
	int result;

	// Compute cost estimate to phase 1 goal state
	cost = Phase1Cost(twist, flip, choice);	// h

	if (cost == 0)	// Phase 1 solution found...
	{
		solutionLength1 = depth;	// Save phase 1 solution length

		// We need an appropriately initialized cube in order
		//   to begin phase 2.  First, create a new cube that
		//   is a copy of the initial scrambled cube.  Then we
		//   apply the phase 1 move sequence to that cube.  The
		//   phase 2 search can then determine the initial
		//   phase 2 coordinates (corner, edge, and slice
		//   permutation) from this cube.
		//
		//   Note: No attempt is made to merge moves of the same
		//   face adjacent to the phase 1 & phase 2 boundary since
		//   the shorter sequence will quickly be found.
		
		KociembaCube phase2Cube = cube;
		for (int i = 0; i < solutionLength1; i++)
		{
			for (power = 0; power < solutionPowers1[i]; power++) {
				phase2Cube.ApplyMove(solutionMoves1[i]);
				if (elapsed_time.check() > timeout){
					return TIMEOUT;
				}
			}
		}
		// Invoke Phase 2
		(void)Solve2(phase2Cube);
	}

	// See if node should be expanded
	totalCost = depth + cost;	// g + h

	if (totalCost <= threshold1)	// Expand node
	{
		// If this happens, we should have found the
		//   optimal solution at this point, so we
		//   can exit indicating such.  Note: the first
		//   complete solution found in phase1 is optimal
		//   due to it being an addmissible IDA* search.
		if (depth >= minSolutionLength-1)
			return OPTIMUM_FOUND;
		
		for (move = Cube::R; move <= Cube::B; move++)
		{
			if (Disallowed(move, solutionMoves1, depth)) continue;
				
			twist2  = twist;
			flip2   = flip;
			choice2 = choice;

			solutionMoves1[depth] = move;
			for (power = 1; power < 4; power++)
			{
				solutionPowers1[depth] = power;
				twist2  = twistMoveTable[twist2][move];
				flip2   = flipMoveTable[flip2][move];
				choice2 = choiceMoveTable[choice2][move];
				nodes1++;
				// Apply the move
				if (elapsed_time.check() > timeout){
					return TIMEOUT;
				}
				if ( (result = Search1(twist2, flip2, choice2, depth+1)) )
						return result;
			}
		}
	}
	else	// Maintain minimum cost exceeding threshold
	{
		if (totalCost < newThreshold1)
			newThreshold1 = totalCost;
	}
	return NOT_FOUND;
}

int Solver::Solve2(KociembaCube& cube)
{
	int iteration = 1;
	int result = NOT_FOUND;

	// Establish initial cost estimate to goal state
	threshold2 = Phase2Cost(
		cube.CornerPermutation(),
		cube.NonMiddleSliceEdgePermutation(),
		cube.MiddleSliceEdgePermutation());

	nodes2 = 1;		// Count root node here
	solutionLength2 = 0;

	do
	{
		if (elapsed_time.check() > timeout){
			return TIMEOUT;
		}

		newThreshold2 = Huge;	// Any cost will be less than this

		// Perform the phase 2 recursive IDA* search
		result = Search2(
			cube.CornerPermutation(),
			cube.NonMiddleSliceEdgePermutation(),
			cube.MiddleSliceEdgePermutation(), 0);

		// Establish a new threshold for a deeper search
		threshold2 = newThreshold2;
		
		// Count interative deepenings
		iteration++;
	} while (result == NOT_FOUND);

	return result;
}

int Solver::Search2(
	int cornerPermutation,
	int nonMiddleSliceEdgePermutation,
	int middleSliceEdgePermutation,
	int depth)
{
	if (elapsed_time.check() > timeout){
		return TIMEOUT;
	}
	int cost, totalCost;
	int move;
	int power, powerLimit;
	int	cornerPermutation2;
	int nonMiddleSliceEdgePermutation2;
	int middleSliceEdgePermutation2;
	int result;

	// Compute cost estimate to goal state
	cost = Phase2Cost(
		cornerPermutation,
		nonMiddleSliceEdgePermutation,
		middleSliceEdgePermutation);	// h

	if (cost == 0)	// Solution found...
	{
		solutionLength2 = depth;	// Save phase 2 solution length
		if (solutionLength1 + solutionLength2 < minSolutionLength) {
			minSolutionLength = solutionLength1 + solutionLength2;

			// Backup copy of best solution...
			memcpy(bestSolutionMoves1, solutionMoves1, sizeof(solutionMoves1));
			memcpy(bestSolutionMoves2, solutionMoves2, sizeof(solutionMoves2));
			memcpy(bestSolutionPowers1, solutionPowers1, sizeof(solutionPowers1));
			memcpy(bestSolutionPowers2, solutionPowers2, sizeof(solutionPowers2));
			bestSolutionLength1 = solutionLength1;
			bestSolutionLength2 = solutionLength2;

			PrintSolution();
		}
		return FOUND;
	}

	// See if node should be expanded
	totalCost = depth + cost;	// g + h

	if (totalCost <= threshold2)	// Expand node
	{
		// No point in continuing to search for solutions of equal or greater
		//   length than the current best solution
		if (solutionLength1 + depth >= minSolutionLength-1) return ABORT;

		for (move = Cube::R; move <= Cube::B; move++)
		{
			if (Disallowed(move, solutionMoves2, depth)) continue;

			cornerPermutation2 = cornerPermutation;
			nonMiddleSliceEdgePermutation2 = nonMiddleSliceEdgePermutation;
			middleSliceEdgePermutation2 = middleSliceEdgePermutation;

			solutionMoves2[depth] = move;
			powerLimit = 4;
			if (move != Cube::U && move != Cube::D) powerLimit=2;

			for (power = 1; power < powerLimit; power++)
			{
				if (elapsed_time.check() > timeout){
					return TIMEOUT;
				}
				cornerPermutation2 =
					cornerPermutationMoveTable[cornerPermutation2][move];
				nonMiddleSliceEdgePermutation2 =
					nonMiddleSliceEdgePermutationMoveTable[nonMiddleSliceEdgePermutation2][move];
				middleSliceEdgePermutation2 =
					middleSliceEdgePermutationMoveTable[middleSliceEdgePermutation2][move];

				solutionPowers2[depth] = power;

				nodes2++;
				// Apply the move
				if ( ( result = Search2(
						cornerPermutation2,
						nonMiddleSliceEdgePermutation2,
						middleSliceEdgePermutation2, depth+1)) )
						return result;
			}
		}
	}
	else	// Maintain minimum cost exceeding threshold
	{
		if (totalCost < newThreshold2)
			newThreshold2 = totalCost;
	}
	return NOT_FOUND;
}

int Solver::Phase1Cost(int twist, int flip, int choice)
{
	// Combining admissible heuristics by taking their maximum
	//   produces an improved admissible heuristic.
	int cost = TwistAndFlipPruningTable.GetValue(twist*flipMoveTable.SizeOf()+flip);
	int cost2 = TwistAndChoicePruningTable.GetValue(twist*choiceMoveTable.SizeOf()+choice);
	if (cost2 > cost) cost = cost2;
	cost2 = FlipAndChoicePruningTable.GetValue(flip*choiceMoveTable.SizeOf()+choice);
	if (cost2 > cost) cost = cost2;
	return cost;
}

int Solver::Phase2Cost(
		int cornerPermutation,
		int nonMiddleSliceEdgePermutation,
		int middleSliceEdgePermutation)
{
	// Combining admissible heuristics by taking their maximum
	//   produces an improved admissible heuristic.
	int cost = CornerAndSlicePruningTable.GetValue(
		cornerPermutation*middleSliceEdgePermutationMoveTable.SizeOf()+middleSliceEdgePermutation);
	int cost2 = EdgeAndSlicePruningTable.GetValue(
		nonMiddleSliceEdgePermutation*middleSliceEdgePermutationMoveTable.SizeOf()+middleSliceEdgePermutation);
	if (cost2 > cost) cost = cost2;
	return cost;
}

int Solver::Disallowed(int move, int* solutionMoves, int depth)
{
	if (depth > 0)
	{
		// Disallow successive moves of a single face (RR2 is same as R')
		if (solutionMoves[depth-1] == move)
			return 1;

		//   Disallow a move of an opposite face if the current face
		//     moved is B,L, or D. (BF, LR, DU are same as FB,RL,UD)
		if ((move == Cube::F) && solutionMoves[depth-1] == Cube::B)
			return 1;
		if ((move == Cube::R) && solutionMoves[depth-1] == Cube::L)
			return 1;
		if ((move == Cube::U) && solutionMoves[depth-1] == Cube::D)
			return 1;

		// Disallow 3 or more consecutive moves of opposite faces
		//   (UDU is same as DU2 and U2D)
		if ((depth > 1) && solutionMoves[depth-2] == move &&
			solutionMoves[depth-1] == Cube::OpposingFace(move))
			return 1;
	}
	return 0;	// This move is allowed
}

void Solver::PrintSolution(void)
{
	int i;
	std::stringstream strout;
	std::string str;
	for (i = 0; i < solutionLength1; i++) {
		str.append(Cube::NameOfMove(TranslateMove(solutionMoves1[i], solutionPowers1[i], 0)));
		str.append(" ");
	}
	str.append(" . ");
	for (i = 0; i < solutionLength2; i++) {
		str.append(Cube::NameOfMove(TranslateMove(solutionMoves2[i], solutionPowers2[i], 1)));
		str.append( " ");
	}
	ROS_INFO("Intermediate solution: %s",str.c_str());
}

std::vector<std::string> Solver::GetSolution(void)
{
	std::vector<std::string> solution;
	int i;
	std::stringstream strout;
	std::string str;

    // New bit from Chris: If the end of phase 1 solution matches the beginning of phase two solution,
    // then they need to be merged into one sensible action.....
 	if (bestSolutionMoves1[bestSolutionLength1-1] == bestSolutionMoves2[0]) {
//		std::cout <<"WARN WARN WARN: Need to merge parts or not a sensible solution!\n";
//		std::cout << "E: " << bestSolutionMoves1[bestSolutionLength1-1] << "   B: " <<bestSolutionMoves2[0] << std::endl;
//		std::cout << "Ep: " << bestSolutionPowers1[bestSolutionLength1-1] << "   Bp: " <<bestSolutionPowers2[0] << std::endl;
		int power = bestSolutionPowers2[0];
		if (bestSolutionMoves2[0] != Cube::U && bestSolutionMoves2[0] != Cube::D)
			power = 2;
		bestSolutionPowers1[bestSolutionLength1-1] += power;
		memcpy(bestSolutionPowers2,bestSolutionPowers2+1,sizeof(int)*31);
		memcpy(bestSolutionMoves2,bestSolutionMoves2+1,sizeof(int)*31);
		bestSolutionLength2-=1;
	}

	for (i = 0; i < bestSolutionLength1; i++) {
		str.append(Cube::NameOfMove(TranslateMove(bestSolutionMoves1[i], bestSolutionPowers1[i], 0)));
		solution.push_back(str);
		str.clear();
	}
	for (i = 0; i < bestSolutionLength2; i++) {
		str.append(Cube::NameOfMove(TranslateMove(bestSolutionMoves2[i], bestSolutionPowers2[i], 1)));
		solution.push_back(str);
		str.clear();
	}
	return solution;
}

int Solver::TranslateMove(int move, int power, int phase2)
{
	int translatedMove = move;

	if (phase2 && move != Cube::U && move != Cube::D)
		power = 2;
		
	if (power == 2)
		translatedMove = Cube::QuarterTurnToHalfTurnMove(move);
	else if (power == 3)
		translatedMove = Cube::InverseOfMove(move);

	return translatedMove;
}
