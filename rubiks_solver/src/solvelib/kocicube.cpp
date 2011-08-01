// kocicube.cpp - Rubik's Cube class implementation
//   with extensions for Kociemba's algorithm

#include "kocicube.h"	// KociembaCube
#include "combinat.h"   // Combinatorial algorithms
#include "vector.h"     // Vector manipulation routines

// Default constructor
KociembaCube::KociembaCube(void)
{
	// Initialize the choice lookup tables (Dik Winter's method)
#ifndef KOCIEMBA
	InitializeChoice();
#endif
}

// Destructor
KociembaCube::~KociembaCube(void)
{
	// Nothing to do here
}

// Corner orientations
//   Twist is represented as a trinary (base 3) number
int KociembaCube::Twist(void)
{
	int corner;	// The current corner
	int twist = 0;
	for (corner = FirstCornerCubie; corner < LastCornerCubie; corner++)
		twist = twist*3 + CornerCubieOrientations[corner];

	return twist;
}

void KociembaCube::Twist(int twist)
{
	int corner;	// The current corner
	int paritySum = 0;	// For calculating corner parity
	for (corner =  LastCornerCubie-1; corner >= FirstCornerCubie; corner--)
	{
		paritySum += (CornerCubieOrientations[corner] = twist%3);
		twist /= 3;
	}
	// Derive the flip of the last edge cubie
	//   from the current total corner parity.
	//   (total corner parity must be a multiple of 3)
	// 3-ParitySum%3 is the amount to add to round up
	//   to the next multiple of 3.  Note that in the case
	//   where ParitySum%3 equals zero we want the amount
	//   to be zero, not three.  Therefore we compute
	//   (3-ParitySum%3)%3.
	CornerCubieOrientations[LastCornerCubie] = (3-paritySum%3)%3;
	return;
}

// Edge orientations
//   Flip is represented as a binary number
int KociembaCube::Flip(void)
{
	int edge;	// The current edge
	int flip = 0;
	for (edge = FirstEdgeCubie; edge < LastEdgeCubie; edge++)
		flip = flip*2 + EdgeCubieOrientations[edge];

	return flip;
}

void KociembaCube::Flip(int flip)
{
	int edge;	// The current edge
	int paritySum = 0;	// For calculating edge parity
	for (edge =  LastEdgeCubie-1; edge >= FirstEdgeCubie; edge--)
	{
		paritySum += (EdgeCubieOrientations[edge] = flip%2);
		flip /= 2;
	}
	// Derive the flip of the last edge cubie
	//   from the current total edge parity
	//   (total edge parity must be even)
	EdgeCubieOrientations[LastEdgeCubie] = paritySum%2;
	return;
}

// Choice of the four middle slice edge positions
// Note that "choice" is for the permutation of those edge
//    cubicles occupied by middle slice edge cubies, as opposed to
//    a choice of all edge cubies within edge cubicles.
int KociembaCube::Choice(void)
{
	int choicePermutation[4];	// Permutation of the four middle slice edges
	int edge;		// The current edge
	int i = 0;
	// Scan for middle slice edges to construct the choice permutation vector
	for (edge =  FirstEdgeCubie; edge <= LastEdgeCubie; edge++)
	{
		if (IsMiddleSliceEdgeCubie(EdgeCubiePermutations[edge]))
			choicePermutation[i++] = edge;
	}
	return ChoiceOrdinal(choicePermutation);
}

void KociembaCube::Choice(int choice)
{
	ChoicePermutation(choice, EdgeCubiePermutations);
//	PrintVector(EdgeCubiePermutations, 12);
	return;
}

// Permutation of the 8 corners
int KociembaCube::CornerPermutation(void)
{
	return PermutationToOrdinal(CornerCubiePermutations, NumberOfCornerCubies);
}

void KociembaCube::CornerPermutation(int ordinal)
{
	OrdinalToPermutation(ordinal, CornerCubiePermutations, NumberOfCornerCubies, FirstCornerCubie);
	return;
}

// Permutation of the 8 non-middle slice edges
int KociembaCube::NonMiddleSliceEdgePermutation(void)
{
	return PermutationToOrdinal(EdgeCubiePermutations, 8);
}

// Note: None of the non middle slice edge cubies are
//   allowed to be in the middle slice prior to calling
//   this function.  If that is not the case, then you
//   must first call Cube::BackToHome().
void KociembaCube::NonMiddleSliceEdgePermutation(int ordinal)
{
//	PrintVector(EdgeCubiePermutations, 8);
	OrdinalToPermutation(ordinal, EdgeCubiePermutations, 8, FirstEdgeCubie);
	return;
}

// Permutation of the 4 middle slice edges
int KociembaCube::MiddleSliceEdgePermutation(void)
{
//	PrintVector(&EdgeCubiePermutations[EdgeCubie::FirstMiddleSliceEdgeCubie], 4);
	return PermutationToOrdinal(&EdgeCubiePermutations[FirstMiddleSliceEdgeCubie], 4);
}

// Note: All of the middle slice edge cubies must be in
//   the middle slice prior to calling prior to calling
//   If that is not the case, then you must first call
//   Cube::BackToHome().
void KociembaCube::MiddleSliceEdgePermutation(int ordinal)
{
	OrdinalToPermutation(ordinal, &EdgeCubiePermutations[FirstMiddleSliceEdgeCubie], 4, FirstMiddleSliceEdgeCubie);
	return;
}

// Predicate to determine if a cubie is a middle slice edge cubie
int KociembaCube::IsMiddleSliceEdgeCubie(int cubie)
{
	return cubie >= FirstMiddleSliceEdgeCubie && cubie <= LastMiddleSliceEdgeCubie;
}

#ifdef KOCIEMBA

// The following algorithm implements the approach taken by Herbert Kociemba
// to map the permutation of the middle slice edges to a unique ordinal within
// the range (0..494).  This approach does not use much memory as no lookup
// tables are employed and it is reasonably efficient.

// ChoiceOrdinal - Compute a unique ordinal for each of the 495
//   (i.e. 12 Choose 4) possible middle slice edge permutations.
//   This is simply referred to as "choice".  The ordinal is in
//   the range (0...494) where 0 corresponds to the [0,1,2,3] edge
//   permutation and 494 corresponds to the [8,9,10,11] permutation.
//   The algorithm below is best understood by a simple example.
//
//   Consider 6 edges taken 3 at a time.  In lexicographic order,
//   the permutations are:
//
//    0) 012	5 Choose 2 = 10 possibilites beginning with 0
//    1) 013		4 Choose 1 = 4 possibilities beginning with 01
//    2) 014
//    3) 015
//    4) 023		3 Choose 1 = 3 possibilities beginning with 02
//    5) 024
//    6) 025
//    7) 034		2 Choose 1 = 2 possibilities beginning with 03
//    8) 035
//    9) 045		1 Choose 1 = 1 possibility beginning with 04
//
//   10) 123	4 Choose 2 = 6 possibities beginning with 1
//   11) 124		3 Choose 1 = 3 possibilities beginning with 12
//   12) 125
//   13) 134		2 Choose 1 = 2 possibilities beginning with 13
//   14) 135
//   15) 145		1 Choose 1 = 1 possibility beginning with 14
//
//   16) 234	3 Choose 2 = 3 possibilites beginning with 2
//   17) 235		2 Choose 1 = 2 possibilities beginning with 23
//   18) 245		1 Choose 1 = 1 possibility beginning with 24
//
//   19) 345	2 Choose 2 = 1 possibility beginning with 3
//
//  Since each permutation is monotonically increasing, it's easy to see
//  how to determine the number of possibilities for any given permutation
//  prefix.  All edge permutations to the right of a given edge permutations
//  must be greater than the given permutation number, so the number of
//  remaining choices can be reduced accordingly at that point (e.g. if
//  edge 1 is present then we are limited to choices 2,3,4,5 in the remaining
//  two positions thus there are 4 choose 2 = 6 permutations beginning with
//  1.
//  The edges are first sorted, using a radix sort (see the mark vector
//  below).  The edges are then scanned in ascending (lexicographic)
//  order.  If an edge is not present, then the ordinal is increased by
//  the number of possible permutations for the current edge choices.
//
int KociembaCube::ChoiceOrdinal(int* choicePermutation)
{
	int edgeMarkVector[NumberOfEdgeCubies];		// For radix sort of the edges
	int edgesRemaining = 4;		// Counts remaining edges
	int ordinal = 0;		// The choice permutation ordinal
	int edge;				// The current edge

	// Radix sort the edges
	for (edge = 0; edge < NumberOfEdgeCubies; edge++)
		edgeMarkVector[edge] = 0;
	for (edge = 0; edge < 4; edge++)
		edgeMarkVector[choicePermutation[edge]] = 1;

	// Scan the edges and compute the ordinal for this permutation
	edge = 0;
	while (edgesRemaining > 0)
	{
		if (edgeMarkVector[edge++])
			edgesRemaining--;	// One less edge to go
		else
			// Skip this many permutations
			ordinal += NChooseM(12-edge, edgesRemaining-1);
	}
	return ordinal;
}

// ChoicePermutation - Given a choice ordinal, compute the associated
//   choice permutation.  Cubicles that are not part of the supplied
//   choice are assigned an invalid cubie.
// The algorithm is essentially the inverse of the permutation to
// choice algorithm.
//
void KociembaCube::ChoicePermutation(int choiceOrdinal, int* choicePermutation)
{
	int edge;		// The current edge
	int digit = 0;		// The currend edge permutation "digit"
	int combinations;	// Number of combinations prefixed with this "digit"

	// All other edges are unknown, so begin by initializing them to "invalid"
	for (edge = 0; edge < NumberOfEdgeCubies; edge++)
		choicePermutation[edge] = InvalidCubie;

	// Advance four "digits"
	for (edge = 0; edge < 4; edge++)
	{
		// This is something like division where we divide by subtracting
		// off the number of combinations possible for the current "digit".
		for (;;)
		{
			// Initially starting at 0###, so this begins at 11 Choose 3
			//   (0 is eliminated leaving 11 possibilites, and there are
			//    3 unassigned "digits")
			// N decreases each time we advance the "digit"
			// M decreases each time we move one "digit" to the right
			combinations=NChooseM(12-1-digit++, 4-1-edge);
			if (choiceOrdinal >= combinations)
				choiceOrdinal -= combinations;
			else
				break;
		}
		// Since digit is always bumped, must back up by one
		// Assign middle slice edges in ascending order
		choicePermutation[digit-1] = FirstMiddleSliceEdgeCubie+edge;
	}
}

#else // Dik Winter's method

//
//  And here is an alternate approach...
//

// The following algorithm implements the approach taken by Dik Winter to
// map the permutation of the middle slice edges to a unique ordinal within
// the range (0..494).  This approach is very fast as it uses lookup tables.
// The lookup tables are not unreasonably large.

// Maps the edge permutations to an ordinal
int KociembaCube::EncodedChoicePermutationToChoiceOrdinal[NumberOfEncodedChoicePermutations];
// Maps the ordinal back to the edge permutation
int KociembaCube::ChoiceOrdinalToEncodedChoicePermutation[NumberOfChoiceOrdinals];

void KociembaCube::DecodeChoicePermutation(int encodedChoicePermutation, int* decodedChoicePermutation)
{
	int element;
	int edge = 0;	// The current edge
	for (element = 0; element < NumberOfEdgeCubies; element++)
	{
		// Expand the edge bits into an integer vector
		decodedChoicePermutation[element] = InvalidCubie;
		if (encodedChoicePermutation&1 == 1)
			decodedChoicePermutation[element] = FirstMiddleSliceEdgeCubie+edge++;
		encodedChoicePermutation >>= 1;
	}
}

// InitializeChoice - Initialize the lookup tables for mapping edge choice permutations
//   to choice ordinal (and vice versa).
//
// Here's how it works:
//
// Consider a bit array of length 12.  Each of the twelve bits represents one of the 12
// 12 edge locations.  Now treat the bit array as an integer in the range (0...4095) and
// cycle through the entire range.  There will be exactly 495 (12 choose 4) values of
// this integer where exactly four bits (i.e. edges) are turned on.  Whenever one of these
// values is detected, assign it to an ordinal and then increment the ordinal since each
// of these values represents a unique permutation of four edges.  We can use these values
// to create two lookup tables, one which maps the edge permutations to an ordinal, the
// other which maps the ordinal back to the edge permutation.  Thus we have our mapping.
//
void KociembaCube::InitializeChoice()
{
	int encodedChoicePermutation;		// Encodes the edge locations, 1 bit per edge
	int choiceOrdinal = 0;				// Ordinal to be assigned to a permutation of four edges
	// Cycle through all possible 12 bit values
	for (encodedChoicePermutation = 0;
	     encodedChoicePermutation < NumberOfEncodedChoicePermutations;
	     encodedChoicePermutation++)
	{
		// If exactly four edges present, then we found a unique permutation of four edges
		if (CountBits(encodedChoicePermutation) == 4)
		{
			// Add entry to choice permutation to choice ordinal lookup table
			EncodedChoicePermutationToChoiceOrdinal[encodedChoicePermutation] = choiceOrdinal;
			// Add entry to choice ordinal to choice permutation lookup table
			ChoiceOrdinalToEncodedChoicePermutation[choiceOrdinal] = encodedChoicePermutation;
			choiceOrdinal++;	// Advance ordinal for next permutation
		}
	}
}

// Count the number of bits set in "value"
int KociembaCube::CountBits(int value)
{
	int sum = 0;
	while (value)
	{
		sum += value&1;
		value >>= 1;
	}
	return sum;
}

// Maps a bit number in the range (0...12) to a bit mask
unsigned int KociembaCube::BitNumberToBitMask[12] = {
	1<< 0, 1<< 1, 1<< 2, 1<< 3, 1<< 4, 1<< 5,
	1<< 6, 1<< 7, 1<< 8, 1<< 9, 1<<10, 1<<11 };

// ChoiceOrdinal - Compute a unique ordinal for each of the 495
//   (i.e. 12 Choose 4) possible middle slice edge permutations.
//   This is simply referred to as "choice".  The ordinal is in
//   the range (0...494).
//
int KociembaCube::ChoiceOrdinal(int* choicePermutation)
{
	// Map each permutation (of edge cubicles containing a middle slice edge) to
	// the corresponding edge bits to construct an encoded edge choice permutation.
	// Then lookup the associated choice ordinal.
	return EncodedChoicePermutationToChoiceOrdinal[
		BitNumberToBitMask[choicePermutation[0]]
	      | BitNumberToBitMask[choicePermutation[1]]
	      | BitNumberToBitMask[choicePermutation[2]]
	      | BitNumberToBitMask[choicePermutation[3]] ];
}

// ChoicePermutation - Given a choice ordinal, compute the associated
//   choice permutation.  Cubicles that are not part of the supplied
//   choice are assigned an invalid cubie.
// Implemented via lookup table.
//
void KociembaCube::ChoicePermutation(int choiceOrdinal, int* choicePermutation)
{
	DecodeChoicePermutation(ChoiceOrdinalToEncodedChoicePermutation[choiceOrdinal],
				choicePermutation);
}

#endif // KOCIEMBA
