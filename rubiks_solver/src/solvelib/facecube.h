// facecube.h - Facelet Cube class definition
//
// The FaceletCube represents the cube by the markings
// of the 54 individual facelets.  The FaceletCube can
// then be asked to validate the cube to determine if
// it is in a legal, and thus solvable, configuration.
//
#ifndef	_facecube_h
#define	_facecube_h

#include "cube.h"		// Cube

// The following macros are used to construct a unique number
// for each corner and edge cubie from its facelets.
//
#define FacesToCorner(face1, face2, face3) (((face1*6)+face2)*6+face3)
#define FacesToEdge(face1, face2) (face1*6+face2)

class FaceletCube
{
public:
	// Face names
	enum Face { U, D, L, R, F, B };

	// Validation return codes
	enum {
		VALID,
		INVALID_MARKER,
		INVALID_FACELETCOUNT,
		DUPLICATE_CENTER_MARKING,
		INVALID_CORNER_MARKINGS,
		INVALID_CORNER_PARITY,
		INVALID_EDGE_MARKINGS,
		INVALID_EDGE_PARITY,
		INVALID_TOTAL_PARITY,
		NumberOfErrors };

		FaceletCube(void);
		~FaceletCube();

	// Set the cube markings for a given face
	void SetFaceMarkings(int face, const char* faceMarkings);

	// Validate markings, permutation, and parity
	int Validate(Cube& cube);

	// Convert face name to enumeration offset
	int FaceNameToOffset(char faceName);

	// Return the text associated with an error return code
	char* ErrorText(unsigned int error);

	// Dump cube state
	void Dump(void);

private:
	// Map each corner facelet to a unique corner number
	enum Corner {
		URF = FacesToCorner(U,R,F),
		RFU = FacesToCorner(R,F,U),
		FUR = FacesToCorner(F,U,R),

		UFL = FacesToCorner(U,F,L),
		FLU = FacesToCorner(F,L,U),
		LUF = FacesToCorner(L,U,F),

		ULB = FacesToCorner(U,L,B),
		LBU = FacesToCorner(L,B,U),
		BUL = FacesToCorner(B,U,L),

		UBR = FacesToCorner(U,B,R),
		BRU = FacesToCorner(B,R,U),
		RUB = FacesToCorner(R,U,B),

		DFR = FacesToCorner(D,F,R),
		FRD = FacesToCorner(F,R,D),
		RDF = FacesToCorner(R,D,F),

		DLF = FacesToCorner(D,L,F),
		LFD = FacesToCorner(L,F,D),
		FDL = FacesToCorner(F,D,L),

		DBL = FacesToCorner(D,B,L),
		BLD = FacesToCorner(B,L,D),
 		LDB = FacesToCorner(L,D,B),

		DRB = FacesToCorner(D,R,B),
		RBD = FacesToCorner(R,B,D),
		BDR = FacesToCorner(B,D,R) };

	// Map each edge facelet to a unique edge number
	enum Edge {
		UF = FacesToEdge(U,F),
		FU = FacesToEdge(F,U),

		UL = FacesToEdge(U,L),
		LU = FacesToEdge(L,U),

		UB = FacesToEdge(U,B),
		BU = FacesToEdge(B,U),
		
		UR = FacesToEdge(U,R),
		RU = FacesToEdge(R,U),
		
		DF = FacesToEdge(D,F),
		FD = FacesToEdge(F,D),
		
		DL = FacesToEdge(D,L),
		LD = FacesToEdge(L,D),
		
		DB = FacesToEdge(D,B),
		BD = FacesToEdge(B,D),
		
		DR = FacesToEdge(D,R),
		RD = FacesToEdge(R,D),
		
		RF = FacesToEdge(R,F),
		FR = FacesToEdge(F,R),
		
		LF = FacesToEdge(L,F),
		FL = FacesToEdge(F,L),
		
		LB = FacesToEdge(L,B),
		BL = FacesToEdge(B,L),
		
		RB = FacesToEdge(R,B),
		BR = FacesToEdge(B,R) };

	// Validation sub functions
	int ValidateCenters(void);
	int ValidateFacelets(void);
	int ValidateCorners(void);
	int ValidateEdges(void);
	int EdgePermutationParity(void);
	int CornerPermutationParity(void);
	int PermutationParity(int* permutations, int numberOfCubies);
	int FaceletOffsetToMarking(int offset);

	// The 9 markings for each of the 6 faces
	char faceletMarkings[6*9];
	// Markings mapped to each face
	char faceMarkings[6+1];

	static int cornerFacelets[Cube::NumberOfCornerCubies][3];
	static Corner cornerMap[Cube::NumberOfCornerCubies*3];

	static int edgeFacelets[Cube::NumberOfEdgeCubies][2];
	static Edge edgeMap[Cube::NumberOfEdgeCubies*2];

	// The resulting cubie permutation and orientations
	int cornerCubiePermutations[Cube::NumberOfCornerCubies];
	int cornerCubieOrientations[Cube::NumberOfCornerCubies];

	int edgeCubiePermutations[Cube::NumberOfEdgeCubies];
	int edgeCubieOrientations[Cube::NumberOfEdgeCubies];

	static char* errorText[NumberOfErrors];
};

#endif // _facecube_h
