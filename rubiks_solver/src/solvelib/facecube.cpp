// facecube.cpp - Facelet Cube class implementation

#include "facecube.h"	// FaceCube
#include "vector.h"     // Vector manipulation routines

#include <string.h>     // For strchr()

FaceletCube::FaceletCube(void)
{
	faceMarkings[6] = '\0';	// Nul terminate
}

FaceletCube::~FaceletCube()
{
	// Nothing to do here
}

// Set the cube markings for a given face
void FaceletCube::SetFaceMarkings(int face, const char* markings)
{
	int facelet;
	for (facelet = 0; facelet < 9; facelet++)
		faceletMarkings[face*9+facelet] = markings[facelet];
}

// Validate markings, permutation, and parity
int FaceletCube::Validate(Cube& cube)
{
	int status;
	
	// Must validate centers first!
	
	if ((status = ValidateCenters()) != VALID)
		return status;

	if ((status = ValidateFacelets()) != VALID)
		return status;
	
	if ((status = ValidateCorners()) != VALID)
		return status;

	if ((status = ValidateEdges()) != VALID)
		return status;

	// Total corner permutation parity must equal
	//   total edge permutation parity
	if (EdgePermutationParity() != CornerPermutationParity())
		return INVALID_TOTAL_PARITY;

	// Return a properly initialized cube model
	cube.SetState(cornerCubiePermutations, cornerCubieOrientations,
		edgeCubiePermutations, edgeCubieOrientations);

	return status;
}

// Dump cube state
void FaceletCube::Dump(void)
{
	PrintVector(cornerCubiePermutations, Cube::NumberOfCornerCubies);
	PrintVector(cornerCubieOrientations, Cube::NumberOfCornerCubies);
	PrintVector(edgeCubiePermutations, Cube::NumberOfEdgeCubies);
	PrintVector(edgeCubieOrientations, Cube::NumberOfEdgeCubies);
}

int FaceletCube::ValidateCenters(void)
{
	int face;
	int faceName;

	// Initialize face marking
	for (face = 0; face < 6; face++) faceMarkings[face] = '\0';
	
	// Establish face markings and make sure each face marking
	//   is unique
	for (face = 0; face < 6; face++)
	{
		for (faceName = 0; faceName < 6; faceName++)
		{
			if (faceMarkings[faceName] == faceletMarkings[face*9+4])
				return DUPLICATE_CENTER_MARKING;	// Duplicate!
		}
		faceMarkings[face] = faceletMarkings[face*9+4];	// Set another center face
	}
	return VALID;
}

int FaceletCube::ValidateFacelets(void)
{
	int face;
	int facelet;
	int facelets[6];

	// Reset facelet count for all faces
	for (face = 0; face < 6; face++)
		facelets[face] = 0;
	
	// Validate all 54 facelets
	for (facelet = 0; facelet < 6*9; facelet++)
	{
		if ((face = FaceletOffsetToMarking(facelet)) < 0)
			return INVALID_MARKER;

		facelets[face]++;
	}

	// Each face must containe exactly 9 facelets
	for (face = 0; face < 6; face++)
	{
		if (facelets[face] != 9)
			return INVALID_FACELETCOUNT;
	}
	
	return VALID;
}

int FaceletCube::ValidateCorners(void)
{
	int cubicle;
	int corner, cornerLocation;
	int cornerParity = 0;
	int cubie;

	// For all corner cubies...
	for (cubicle = 0; cubicle < Cube::NumberOfCornerCubies; cubicle++)
	{
		// Compute corner cubie number directly from its facelets
		cubie = FacesToCorner(
			FaceletOffsetToMarking(cornerFacelets[cubicle][0]),
			FaceletOffsetToMarking(cornerFacelets[cubicle][1]),
			FaceletOffsetToMarking(cornerFacelets[cubicle][2]));

		cornerLocation = -1;	// Assume corner cubie "not found"
		// Search the table of valid corner facelet orientations
		for (corner = 0; corner < Cube::NumberOfCornerCubies*3; corner++)
		{
			if (cubie == cornerMap[corner])	// Found a match
			{
				// Save location
				cornerLocation = corner;
				// Permutation corresponds to row offset
				cornerCubiePermutations[cubicle] = corner/3;
				// Orientation corresponds to column offset
				cornerCubieOrientations[cubicle] = corner%3;
				// Accumulate corner parity sum
				cornerParity += cornerCubieOrientations[cubicle];
				break;
			}
		}
		// Error if corner orientation not found
		if (cornerLocation == -1)
			return INVALID_CORNER_MARKINGS;
	}
	
	// Total corner orientation parity (COP) must be zero
	if (cornerParity%3 != 0)
		return INVALID_CORNER_PARITY;
	
	return VALID;
}

int FaceletCube::ValidateEdges(void)
{
	int cubicle;
	int edge, edgeLocation;
	int edgeParity = 0;
	int cubie;

	// For all edge cubies...
	for (cubicle = 0; cubicle < Cube::NumberOfEdgeCubies; cubicle++)
	{
		// Compute edge cubie number directly from its facelets
		cubie = FacesToEdge(
			FaceletOffsetToMarking(edgeFacelets[cubicle][0]),
			FaceletOffsetToMarking(edgeFacelets[cubicle][1]));

		edgeLocation = -1;	// Assume edge cubie "not found"
		// Search the table of valid edge facelet orientations
		for (edge = 0; edge < Cube::NumberOfEdgeCubies*2; edge++)
		{
			if (cubie == edgeMap[edge])	// Found a match
			{
				// Save location
				edgeLocation = edge;
				// Permutation corresponds to row offset
				edgeCubiePermutations[cubicle] = edge/2;
				// Orientation corresponds to column offset
				edgeCubieOrientations[cubicle] = edge%2;
				// Accumulate edge parity sum
				edgeParity += edgeCubieOrientations[cubicle];
				break;
			}
		}
		// Error if edge orientation not found
		if (edgeLocation == -1)
			return INVALID_EDGE_MARKINGS;
	}

	// Total edge orientation parity (EOP) must be zero
	if (edgeParity%2 != 0)
		return INVALID_EDGE_PARITY;

	return VALID;
}

// Compute total edge permutation parity (EPP)
int FaceletCube::EdgePermutationParity()
{
	return PermutationParity(edgeCubiePermutations, Cube::NumberOfEdgeCubies);
}

// Compute total corner permutation parity (CPP)
int FaceletCube::CornerPermutationParity()
{
	return PermutationParity(cornerCubiePermutations, Cube::NumberOfCornerCubies);
}

// Permutation parity can be computed by counting the number of
//   reversals in the permutation sequence, - i.e., the number
//   of pairs (p,q) such that p>q and p precedes q.  Then
//   determine if the result is even or odd.  Do this for both
//   edges (EPP) and corners (CPP). A configuration is reachable
//   if EPP=CPP.  (August/September cube.lovers - Vanderschel/Saxe)
int FaceletCube::PermutationParity(int* permutations, int numberOfCubies)
{
	int p, q;
	int permutationParity = 0;

	for (p = 0; p < numberOfCubies-1; p++)
	{
		for (q = p+1; q < numberOfCubies; q++)
		{
			if (permutations[p] > permutations[q])
				permutationParity++;
		}
	}
	return permutationParity%2;
}

// Convert face name to enumeration offset
int FaceletCube::FaceNameToOffset(char faceName)
{
	static char* faceNames = (char*)"UDLRFB";
	char *name;
	
	if (!(name = strchr(faceNames, faceName)))
		return -1;
	return name-faceNames;	// Compute face enumeration
}

int FaceletCube::FaceletOffsetToMarking(int offset)
{
	char* name;
	if (!(name = strchr(faceMarkings, faceletMarkings[offset])))
		return -1;
	return name-faceMarkings;	// Compute face enumeration
}

/*
 Refer to the diagrams below to determine the array index
 for any given facelet.

 Direction key:

      Up
 Left Front Right Back
      Down

 Facelet indices:

            0  1  2
            3  4  5
            6  7  8

 18 19 20  36 37 38  27 28 29  45 46 47
 21 22 23  39 40 41  30 31 32  48 49 50
 24 25 26  42 43 44  33 34 35  51 52 53

            9 10 11
           12 13 14
           15 16 17
*/
	
// Corner facelet locations
int FaceletCube::cornerFacelets[Cube::NumberOfCornerCubies][3] = {
	{ 8,  27, 38 },		// URF
	{ 6,  36, 20 },		// UFL
	{ 0,  18, 47 },		// ULB
	{ 2,  45, 29 },		// UBR
	{ 11, 44, 33 },		// DFR
	{  9, 26, 42 },		// DLF
	{ 15, 53, 24 },		// DBL
	{ 17, 35, 51 } };	// DRB

// Table of valid corner facelet orientations
FaceletCube::Corner FaceletCube::cornerMap[Cube::NumberOfCornerCubies*3] = {
//   0    1    2	(Twist)
	URF, RFU, FUR,
	UFL, FLU, LUF,
	ULB, LBU, BUL,
	UBR, BRU, RUB,
	DFR, FRD, RDF,
	DLF, LFD, FDL,
	DBL, BLD, LDB,
	DRB, RBD, BDR };

// Edge facelet locations
int FaceletCube::edgeFacelets[Cube::NumberOfEdgeCubies][2] = {
	{  7, 37 },		// UF
	{  3, 19 },		// UL
	{  1, 46 },		// UB
	{  5, 28 },		// UR
	{ 10, 43 },		// DF
	{ 12, 25 },		// DL
	{ 16, 52 },		// DB
	{ 14, 34 },		// DR
	{ 30, 41 },		// RF
	{ 23, 39 },		// LF
	{ 21, 50 },		// LB
	{ 32, 48 } };	// RB

// Table of valid edge facelet orientations
// Marked facelet appears first in the left column below
//   (e.g. U is the the marked facelet of the UF edge, etc.)
FaceletCube::Edge FaceletCube::edgeMap[Cube::NumberOfEdgeCubies*2] = {
//   0   1	(Flip)
	UF, FU,
	UL, LU,
	UB, BU,
	UR, RU,
	DF, FD,
	DL, LD,
	DB, BD,
	DR, RD,
	RF, FR,
	LF, FL,
	LB, BL,
	RB, BR };

// Return the text associated with an error return code
char* FaceletCube::ErrorText(unsigned int error)
{
	if (error >= NumberOfErrors) error = 0;
	return errorText[error];
}

char* FaceletCube::errorText[NumberOfErrors] = {
	(char*)"",
	(char*)"Facelet marking does not match any center marking",
	(char*)"There must be 9 facelets for each marking",
	(char*)"Duplicate center marking",
	(char*)"Invalid corner markings",
	(char*)"Invalid corner orientation parity",
	(char*)"Invalid edge markings",
	(char*)"Invalid edge orientation parity",
	(char*)"Invalid total permutation parity" };
