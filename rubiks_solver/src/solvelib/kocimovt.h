// kocimovt.h - Kociemba's move mapping tables
//
// Subclasses of "MoveTable" for each of the Kociemba
// move mapping tables.  Functions for converting between
// an ordinal and its associated cube state are specified
// for each class via overrides.
//
#include "movetabl.h"
#include "kocicube.h"

// Phase 1 move mapping table classes

class TwistMoveTable : public MoveTable
{
public:
	TwistMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::Twists, 0),
		  TheCube(cube) {}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.Twist(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.Twist(ordinal); }
	KociembaCube& TheCube;
};

class FlipMoveTable : public MoveTable
{
public:
	FlipMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::Flips, 0),
		  TheCube(cube) {}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.Flip(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.Flip(ordinal); }
	KociembaCube& TheCube;
};

class ChoiceMoveTable : public MoveTable
{
public:
	ChoiceMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::Choices, 0),
		  TheCube(cube) {}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.Choice(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.Choice(ordinal); }
	KociembaCube& TheCube;
};

// Phase 2 move mapping table classes

class CornerPermutationMoveTable : public MoveTable
{
public:
	CornerPermutationMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::CornerPermutations, 1),
		  TheCube(cube) {}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.CornerPermutation(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.CornerPermutation(ordinal); }
	KociembaCube& TheCube;
};

class NonMiddleSliceEdgePermutationMoveTable : public MoveTable
{
public:
	NonMiddleSliceEdgePermutationMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::NonMiddleSliceEdgePermutations, 1),
		  TheCube(cube) {}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.NonMiddleSliceEdgePermutation(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.NonMiddleSliceEdgePermutation(ordinal); }
	KociembaCube& TheCube;
};

class MiddleSliceEdgePermutationMoveTable : public MoveTable
{
public:
	MiddleSliceEdgePermutationMoveTable(KociembaCube& cube)
		: MoveTable(cube, KociembaCube::MiddleSliceEdgePermutations, 1),
		  TheCube(cube)	{}
private:
	inline int  OrdinalFromCubeState(void)
		{ return TheCube.MiddleSliceEdgePermutation(); }
	inline void OrdinalToCubeState(int ordinal)
		{ TheCube.MiddleSliceEdgePermutation(ordinal); }
	KociembaCube& TheCube;
};