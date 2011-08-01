// cube.cpp - Rubik's Cube class implementation

#include "cube.h"		// Rubik's Cube class definition
#include "vector.h"		// Vector manipulation routines

#include <string.h>		// For strcmp(), memcmp()

// Default constructor
Cube::Cube(void)
{
	BackToHome();
}

// Destructor
Cube::~Cube()
{
	// Nothing to do here
}

// Overloaded equality test operator
int Cube::operator==(const Cube& cube)
{
	return(
		!memcmp(CornerCubiePermutations, cube.CornerCubiePermutations,
			sizeof(CornerCubiePermutations)) &&
		!memcmp(CornerCubieOrientations, cube.CornerCubieOrientations,
			sizeof(CornerCubieOrientations)) &&
		!memcmp(EdgeCubiePermutations, cube.EdgeCubiePermutations,
			sizeof(EdgeCubiePermutations)) &&
		!memcmp(EdgeCubieOrientations, cube.EdgeCubieOrientations,
			sizeof(EdgeCubieOrientations)) );
}

// Overloaded inequality test operator
int Cube::operator!=(const Cube& cube)
{
	return !(*this == cube);
}

// Reset cube back to HOME position
void Cube::BackToHome(void)
{
	int cubie;
	for (cubie = FirstEdgeCubie; cubie <= LastEdgeCubie; cubie++)
	{
		EdgeCubiePermutations[cubie] = cubie;
		EdgeCubieOrientations[cubie] = NotFlipped;
	}
	for (cubie = FirstCornerCubie; cubie <= LastCornerCubie; cubie++)
	{
		CornerCubiePermutations[cubie] = cubie;
		CornerCubieOrientations[cubie] = NoQuark;
	}
}

// Set state from permutation and orientation vectors
void Cube::SetState(
		int* cornerPermutation, int* cornerOrientation,
		int* edgePermutation, int* edgeOrientation)
{
	int cubie;
	for (cubie = FirstEdgeCubie; cubie <= LastEdgeCubie; cubie++)
	{
		EdgeCubiePermutations[cubie] = edgePermutation[cubie];
		EdgeCubieOrientations[cubie] = edgeOrientation[cubie];
	}
	for (cubie = FirstCornerCubie; cubie <= LastCornerCubie; cubie++)
	{
		CornerCubiePermutations[cubie] = cornerPermutation[cubie];
		CornerCubieOrientations[cubie] = cornerOrientation[cubie];
	}
}
	
// Apply move
void Cube::ApplyMove(int move)
{
	switch (move)
	{
	case Cube::R:	Move_R();	break;
	case Cube::U:	Move_U();	break;
	case Cube::L:	Move_L();	break;
	case Cube::D:	Move_D();	break;
	case Cube::F:	Move_F();	break;
	case Cube::B:	Move_B();	break;
	case Cube::Ri:	Move_Ri();	break;
	case Cube::Li:	Move_Li();	break;
	case Cube::Ui:	Move_Ui();	break;
	case Cube::Di:	Move_Di();	break;
	case Cube::Fi:	Move_Fi();	break;
	case Cube::Bi:	Move_Bi();	break;
	case Cube::R2:	Move_R2();	break;
	case Cube::L2:	Move_L2();	break;
	case Cube::U2:	Move_U2();	break;
	case Cube::D2:	Move_D2();	break;
	case Cube::F2:	Move_F2();	break;
	case Cube::B2:	Move_B2();	break;
	}
}

// Get the move from the move name
int Cube::MoveNameToMove(char* moveName, int& move)
{
	int moveIndex;
	int found = 0;	// Assume "not found"
	// Scan the table of move names looking for a match
	for (moveIndex = 0; moveIndex < NumberOfMoves; moveIndex++)
	{
		if (!strcmp(moveName, MoveNames[moveIndex]))
		{
			move = moveIndex;
			found = 1;
			break;
		}
	}
	return found;
}

// Dump cube state
void Cube::Dump(void)
{
	PrintVector(CornerCubiePermutations, NumberOfCornerCubies);
	PrintVector(CornerCubieOrientations, NumberOfCornerCubies);
	PrintVector(EdgeCubiePermutations, NumberOfEdgeCubies);
	PrintVector(EdgeCubieOrientations, NumberOfEdgeCubies);
}

/*
   "x" denotes the chief facelets of cubicles.  Cube faces not shown are
   symmetrical to the opposing face.
			       ____ ____ ____
			     /_x_ /_x_ /_x_ /|
			   /_x_ /_U_ /_x_ /| |
			 / x  / x  / x  /| |/|
			 ---- ---- ----  |/|x|
			|    |    |    |/|R|/|
			 ---- ---- ---- x|/| |
			|    | F  |    |/| |/
			 ---- ---- ----  |/
			|    |    |    |/
			 ---- ---- ----

  An edge cubie is sane if its chief facelet aligns with the chief
  facelet of its current cubicle, otherwise it is flipped (MetaMagical
  Themas - Hofstadter).

  The orientation of a corner cubie can be determined by the number of
  120 degree counter-clockwise twists required to align its chief
  facelet into a position that is parallel to the chief facelet of
  its home cubicle (August/September cube.lovers - Vanderschel/Saxe)

  The above information was used to implement the following moves.
*/

// Cube moves
void Cube::Move_R(void)
{
	FourCycle(URF, UBR, DRB, DFR);

	ClockwiseTwist       (URF);
	CounterClockwiseTwist(UBR);
	ClockwiseTwist       (DRB);
	CounterClockwiseTwist(DFR);

	FourCycle(UR, BR, DR, RF);

	Flip(UR);	Flip(BR);	Flip(DR);	Flip(RF);
}

void Cube::Move_L(void)
{
	FourCycle(ULB, UFL, DLF, DBL);

	ClockwiseTwist       (ULB);
	CounterClockwiseTwist(UFL);
	ClockwiseTwist       (DLF);
	CounterClockwiseTwist(DBL);

	FourCycle(UL, FL, DL, LB);

	Flip(UL);	Flip(FL);	Flip(DL);	Flip(LB);
}

void Cube::Move_U(void)
{
	FourCycle(ULB, UBR, URF, UFL);

	FourCycle(UB, UR, UF, UL);
}
void Cube::Move_D(void)
{
	FourCycle(DLF, DFR, DRB, DBL);

	FourCycle(DF, DR, DB, DL);
}

void Cube::Move_F(void)
{
	FourCycle(UFL, URF, DFR, DLF);

	ClockwiseTwist       (UFL);
	CounterClockwiseTwist(URF);
	ClockwiseTwist       (DFR);
	CounterClockwiseTwist(DLF);

	FourCycle(UF, RF, DF, FL);
}

void Cube::Move_B(void)
{
	FourCycle(UBR, ULB, DBL, DRB);

	ClockwiseTwist       (UBR);
	CounterClockwiseTwist(ULB);
	ClockwiseTwist       (DBL);
	CounterClockwiseTwist(DRB);

	FourCycle(UB, LB, DB, BR);

}

void Cube::Move_Ri(void)
{
	FourCycle(UBR, URF, DFR, DRB);

	CounterClockwiseTwist(UBR);
	ClockwiseTwist       (URF);
	CounterClockwiseTwist(DFR);
	ClockwiseTwist       (DRB);

	FourCycle(UR, RF, DR, BR);

	Flip(UR);	Flip(RF);	Flip(DR);	Flip(BR);
}

void Cube::Move_Li(void)
{
	FourCycle(UFL, ULB, DBL, DLF);

	CounterClockwiseTwist(UFL);
	ClockwiseTwist       (ULB);
	CounterClockwiseTwist(DBL);
	ClockwiseTwist       (DLF);

	FourCycle(UL, LB, DL, FL);

	Flip(UL);	Flip(LB);	Flip(DL);	Flip(FL);
}

void Cube::Move_Ui(void)
{
	FourCycle(UBR, ULB, UFL, URF);

	FourCycle(UB, UL, UF, UR);
}

void Cube::Move_Di(void)
{
	FourCycle(DFR, DLF, DBL, DRB);

	FourCycle(DF, DL, DB, DR);
}

void Cube::Move_Fi(void)
{
	FourCycle(URF, UFL, DLF, DFR);

	CounterClockwiseTwist(URF);
	ClockwiseTwist       (UFL);
	CounterClockwiseTwist(DLF);
	ClockwiseTwist       (DFR);

	FourCycle(UF, FL, DF, RF);
}

void Cube::Move_Bi(void)
{
	FourCycle(ULB, UBR, DRB, DBL);

	CounterClockwiseTwist(ULB);
	ClockwiseTwist       (UBR);
	CounterClockwiseTwist(DRB);
	ClockwiseTwist       (DBL);

	FourCycle(UB, BR, DB, LB);
}

void Cube::Move_R2(void)
{
	Move_R(); Move_R();
}

void Cube::Move_L2(void)
{
	Move_L(); Move_L();
}

void Cube::Move_U2(void)
{
	Move_U(); Move_U();
}

void Cube::Move_D2(void)
{
	Move_D(); Move_D();
}

void Cube::Move_F2(void)
{
	Move_F(); Move_F();
}

void Cube::Move_B2(void)
{
	Move_B(); Move_B();
}

// Cycle four edge cubies
void Cube::FourCycle(
				EdgeCubie first,
				EdgeCubie second,
				EdgeCubie third,
				EdgeCubie fourth)
{
	CycleFour(EdgeCubiePermutations, first, second, third, fourth);
	CycleFour(EdgeCubieOrientations, first, second, third, fourth);
}

// Cycle four corner cubies
void Cube::FourCycle(
				CornerCubie first,
				CornerCubie second,
				CornerCubie third,
				CornerCubie fourth)
{
	CycleFour(CornerCubiePermutations, first, second, third, fourth);
	CycleFour(CornerCubieOrientations, first, second, third, fourth);
}

// Cycle four cubies
void Cube::CycleFour(
		int* vector,
		int first,
		int second,
		int third,
		int fourth)
{
	int temp       = vector[fourth];
	vector[fourth] = vector[third];
	vector[third]  = vector[second];
	vector[second] = vector[first];
	vector[first]  = temp;
}

// Flip an edge cubie
void Cube::Flip(EdgeCubie cubie)
{
	EdgeCubieOrientations[cubie] ^= 1;
}

// Corner cubie twists
void Cube::ClockwiseTwist(CornerCubie cubie)
{
	// Note: the same effect could be accomplished by:
	//   CornerCubieOrientations[cubie] = (CornerCubieOrientations[cubie]+1)%3;
	//   but for some reason, I prefer the lookup table approach.
	CornerCubieOrientations[cubie] = ClockwiseTwists[CornerCubieOrientations[cubie]];
}

void Cube::CounterClockwiseTwist(CornerCubie cubie)
{
	CornerCubieOrientations[cubie] = CounterClockwiseTwists[CornerCubieOrientations[cubie]];
}

// Move inverses
Cube::Move Cube::InverseMoves[NumberOfMoves] = {
	Ri, Li, Ui, Di, Fi, Bi,
	R,  L,  U,  D,  F,  B,
	R2, L2, U2, D2, F2, B2 };

// Opposing faces
Cube::Move	Cube::OppositeFaces[NumberOfMoves] = {
	L,  R,  D,  U,  B,  F };
	
// Names
char* Cube::MoveNames[NumberOfMoves] = {
	(char*)"R",  (char*)"L",  (char*)"U",  (char*)"D",  (char*)"F",  (char*)"B",
	(char*)"Ri", (char*)"Li", (char*)"Ui", (char*)"Di", (char*)"Fi", (char*)"Bi",
	(char*)"R2", (char*)"L2", (char*)"U2", (char*)"D2", (char*)"F2", (char*)"B2" };

// Corner twist tables
int Cube::ClockwiseTwists[] = {
	Quark, AntiQuark, NoQuark };
int Cube::CounterClockwiseTwists[] = {
	AntiQuark, NoQuark, Quark };
