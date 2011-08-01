// solver.h - Cube Solver class definition
//
// This class performs a two phase IDA* search for a solution
// to the scrambled cube.
//
// Phase 1 searches the group spanned by <U,D,R,L,F,B> until
// a configuration is discovered where the three coordinates
// of twist, flip, and choice are "correct" with respect to a
// solved cube.  This means that no edge cubie is twisted, no
// corner cubie is flipped, and the four middle slice edge
// cubies are in the middle slice (but not necessarily in their
// correct permutation within that slice).  At this point, we
// have found a member of an element of the phase two group.
//
// Phase 2 uses the resulting phase 1 configuration as the
// starting point for a search of the group spanned by
// <U,D,R2,L2,F2,B2>, the goal being to reach <I>, the identity
// (i.e. the solved configuration).  Note that this group
// preserves the three coordinates of the phase 1 search since
// in phase 2 it is impossible to alter the twist, flip, or
// choice aspects of the cube.  The U, D moves do not alter corner
// or edge parity and do not affect the choice of the middle slice
// edge cubies.  The same is true for the R2, L2, F2, B2 moves.
// This can be verified by considering the effect these moves
// have on cube parity (see cube.cpp for details on the parity
// frame of reference used).
//
// In search parlance, the pruning tables (a.k.a. "pattern
// databases) constitute an "admissible" heuristic.  This
// means that they always underestimate the distance (i.e.
// number of moves required) to reach the goal state.  It
// can be proven that any search, such as IDA*, that examines
// nodes at progressively increasing cost values and employs
// an admissible heuristic is "optimal".  This means that the
// first solution found by the search is guaranteed to be of
// minimal distance required to reach the target, or goal,
// state.
// 
// Since the search is split into two sequential IDA* search
// phases, the optimality condition above does not always
// hold.  However, since we allow the phase 1 search to
// iteratively deepen, if let run long enough, it will
// eventually deepen to the point where it is capable of
// finding a complete solution.  At this point, we know we
// have an optimal solution as we have degenerated to a
// a single IDA* search of the cube space, but this takes
// a very long time to occur.  The main strength of the
// two phase search is that it finds a near optimal solution
// very quickly and outputs successively better solutions
// until it eventually finds one that is optimal.  In most
// cases though, the search is terminated early (due to lack
// of patience) once an "adequate" solution is found.
//
// For more information concerning IDA* and admissibility,
// see the paper "Depth-First Iterative-Deepening: An Optimal
// Admissable Tree Search" by Richard E. Korf.  This paper
// appears in volume 25 of "Artificial Intelligence" 1985,
// pp. 97-109.  Also, there are many texts on AI (Artificial
// Intelligence) or books on search techniques that cover
// these topics in depth.
//
#ifndef	_solver_h
#define	_solver_h

#include "kocicube.h"	// KociembaCube
#include "kocimovt.h"	// Kociemba's move mapping tables
#include "pruningt.h"	// PruningTable
#include "Timer.h"
#include <vector>

class Solver
{
public:
	std::vector<int> solution;

	Solver(float timeout = 30.0);
	~Solver();

	// Initializes both the move mapping and pruning tables required
	//   by the search
	void InitializeTables(void);

	// Perform the two phase search
	int Solve(KociembaCube& scrambledCube);
	// Solver return codes
	enum {
		NOT_FOUND,	// A solution was not found
		FOUND,		// A solution was found
		OPTIMUM_FOUND,	// An optimal solution was found
		ABORT,
		TIMEOUT=-1
	};	// The search was aborted
				//   (i.e. phase 2 did not yield an improved solution)

	// Output the solution
	//   Note: you may not need to call this as the best
	//   solution, found so far, is output during the search
	void PrintSolution(void);

	std::vector<std::string> GetSolution(void);


private:

	enum { Huge = 10000 };	// An absurdly large number

	// Initiatates the second phase of the search
	int Solve2(KociembaCube& cube);

	// Phase 1 & 2 recursive IDA* search routines
	int Search1(int twist, int flip, int choice, int depth);
	int Search2(
		int cornerPermutation,
		int nonMiddleSliceEdgePermutation,
		int middleSliceEdgePermutation,
		int depth);

	// Phase 1 & 2 cost heuristics
	int Phase1Cost(int twist, int flip, int choice);
	int Phase2Cost(
		int cornerPermutation,
		int nonMiddleSliceEdgePermutation,
		int middleSliceEdgePermutation);

	// Predicate to determine if a move is redundant (leads to
	//   (a node that is explored elsewhere) and should therefore
	//   be disallowed.
	inline int Disallowed(int move, int* solutionMoves, int depth);

	// Translates moves from a (face, power) representation to a
	//   single move string representation (e.g. R,3 becomes R').
	//   Also if the move was applied during phase 2 and is either
	//   R,L,F, or B, then a power of 2 is assumed.  This is done
	//   since the phase 2 move mapping tables are in terms of half
	//   turn moves for R,L,F, and B and the power used is 1, not 2.
	//   In this way, we do not have to burden the phase 2 search with
	//   determining the correct power for display purposes only.
	//   I hope that's clear.
	//
	int TranslateMove(int move, int power, int phase2);
	
	// Search variables for the two phase IDA* search
	int nodes1, nodes2;				// Number of nodes expanded
	int threshold1, threshold2;                     // Current heuristic threshold (cutoff)
	int newThreshold1, newThreshold2;		// New threshold as determined by current search pass

	int solutionMoves1[32], solutionMoves2[32];	// List of applied moves
	int solutionPowers1[32], solutionPowers2[32];	// List of powers associated with each move
	int solutionLength1, solutionLength2;		// Length of each solution
	int minSolutionLength;				// Minimum solution length found so far

	int bestSolutionMoves1[32], bestSolutionMoves2[32];	// List of applied moves
	int bestSolutionPowers1[32], bestSolutionPowers2[32];	// List of powers associated with each move
	int bestSolutionLength1, bestSolutionLength2;		// Length of each solution

	float timeout;
	Timer elapsed_time;

	// A cube used for two purposes:
	//   1- Initially used by the solver for initializing the move mapping tables
	//   2- Contains a copy of the scrambled cube that is used at the phase 1/phase 2
	///     transition to compute the initial phase 2 coordinates.
	KociembaCube cube;

	// Phase 1 move mapping tables
	TwistMoveTable twistMoveTable;
	FlipMoveTable flipMoveTable;
	ChoiceMoveTable choiceMoveTable;
	// Phase 2 move mapping tables
	CornerPermutationMoveTable cornerPermutationMoveTable;
	NonMiddleSliceEdgePermutationMoveTable nonMiddleSliceEdgePermutationMoveTable;
	MiddleSliceEdgePermutationMoveTable middleSliceEdgePermutationMoveTable;
	
	// Phase 1 pruning tables
	PruningTable TwistAndFlipPruningTable;
	PruningTable TwistAndChoicePruningTable;
	PruningTable FlipAndChoicePruningTable;
	// Phase 2 pruning tables
	PruningTable CornerAndSlicePruningTable;
	PruningTable EdgeAndSlicePruningTable;

//	int solution_size;
};

#endif	// _solver_h
