// cube.h - Rubik's Cube class definition
//
// A cube model containing basic cube definitions and
// cube operations.  Applying a move maintains the
// proper cube permutation and orientation for each
// cubie. This class is designed to be subclassed in
// order to extend it to a specific type of cube
// useful for a particular solution method.
//
#ifndef	_cube_h
#define	_cube_h

class Cube
{
public:
	// Cubie locations
	enum EdgeCubie {
		// Edge locations
		UF =  0, UL =  1, UB =  2, UR =  3,
		FU =  0, LU =  1, BU =  2, RU =  3,

		DF =  4, DL =  5, DB =  6, DR =  7,
		FD =  4, LD =  5, BD =  6, RD =  7,
		// Middle slice edges begin here
		RF =  8, FL =  9, LB = 10, BR = 11,
		FR =  8, LF =  9, BL = 10, RB = 11,

		FirstEdgeCubie = UF,
		LastEdgeCubie = BR,
		FirstMiddleSliceEdgeCubie = RF,
		LastMiddleSliceEdgeCubie = BR,
		NumberOfEdgeCubies = LastEdgeCubie+1 };

	// Corner locations
	enum CornerCubie {
		URF = 0, UFL = 1, ULB = 2, UBR = 3,
		RFU = 0, FLU = 1, LBU = 2, BRU = 3,
		FUR = 0, LUF = 1, BUL = 2, RUB = 3,

		DFR = 4, DLF = 5, DBL = 6, DRB = 7,
		FRD = 4, LFD = 5, BLD = 6, RBD = 7,
		RDF = 4, FDL = 5, LDB = 6, BDR = 7,

		FirstCornerCubie = URF,
		LastCornerCubie = DRB,
		NumberOfCornerCubies = LastCornerCubie+1 };

	// Applies to all cubies
	enum Cubie { InvalidCubie = LastEdgeCubie+1 };

	// Twists
	enum Twists {
		NoQuark, Quark, AntiQuark,
		NumberOfTwists = AntiQuark+1 };
		
	// Flips
	enum Flips {
		NotFlipped, Flipped };

	// Quarter and half turn moves
	enum Move {
		R,  L,  U,  D,  F,  B,
		Ri, Li, Ui, Di, Fi, Bi,
		R2, L2, U2, D2, F2, B2,

		FirstMove = R,
		LastMove = B2,
		NumberOfClockwiseQuarterTurnMoves = B+1,
		NumberOfMoves = LastMove+1 };

	// Default constructor
	Cube(void);

	// Destructor
	virtual ~Cube();

	// Overloaded equality test operator
	int operator==(const Cube& cube);

	// Overloaded inequality test operator
	int operator!=(const Cube& cube);
	
	// Reset cube back to HOME position
	virtual void BackToHome(void);

	// Set state from permutation and orientation vectors
	virtual void SetState(
		int* cornerPermutation, int* cornerOrientation,
		int* edgePermutation, int* edgeOrientation);
	
	// Apply move
	virtual void ApplyMove(int move);

	// Get inverse of move
	inline static int InverseOfMove(int move)
		{ return InverseMoves[move]; }

	// Turns a quarter turn move to a half turn move (e.g. R and Ri become R2)
	inline static int QuarterTurnToHalfTurnMove(int move)
		{ return R2+move%(B+1); }

	// Get opposite face of a move
	inline static int OpposingFace(int move)
		{ return OppositeFaces[move]; };
	
	// Get the name of a move
	inline static char* NameOfMove(int move)
		{ return MoveNames[move]; }

	// Get the move from the move name
	static int MoveNameToMove(char* moveName, int& move);

	// Dump cube state
	virtual void Dump(void);

protected:
	// Cube moves
	void Move_R(void);
	void Move_L(void);
	void Move_U(void);
	void Move_D(void);
	void Move_F(void);
	void Move_B(void);
	void Move_Ri(void);
	void Move_Li(void);
	void Move_Ui(void);
	void Move_Di(void);
	void Move_Fi(void);
	void Move_Bi(void);
	void Move_R2(void);
	void Move_L2(void);
	void Move_U2(void);
	void Move_D2(void);
	void Move_F2(void);
	void Move_B2(void);

	// Cycle four edge cubies
	void FourCycle(
			EdgeCubie first,
			EdgeCubie second,
			EdgeCubie third,
			EdgeCubie fourth);

	// Cycle four corner cubies
	void FourCycle(
			CornerCubie first,
			CornerCubie second,
			CornerCubie third,
			CornerCubie fourth);

	// Cycle four vector elements
	void CycleFour(int* vector,
			int first,
			int second,
			int third,
			int fourth);

	// Flip an edge cubie
	void Flip(EdgeCubie cubie);

	// Corner cubie twists
	void ClockwiseTwist(CornerCubie cubie);
	void CounterClockwiseTwist(CornerCubie cubie);

	// The cubies
	int CornerCubiePermutations[NumberOfCornerCubies];
	int CornerCubieOrientations[NumberOfCornerCubies];

	int EdgeCubiePermutations[NumberOfEdgeCubies];
	int EdgeCubieOrientations[NumberOfEdgeCubies];

private:
	// Twist tables
	static int ClockwiseTwists[NumberOfTwists];
	static int CounterClockwiseTwists[NumberOfTwists];

	// Names
	static char* MoveNames[NumberOfMoves];

	// Move inverse
	static Cube::Move InverseMoves[NumberOfMoves];
	// Opposing faces
	static Cube::Move OppositeFaces[NumberOfMoves];
};

#endif	// _cube_h
