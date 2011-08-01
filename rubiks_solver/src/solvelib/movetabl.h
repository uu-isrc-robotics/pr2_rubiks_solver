// movetabl.h - MoveTable class definition
//
// An abstract base class used for creating move mapping
// tables.  Functions for converting between an ordinal
// and its associated cube state must be overridden in
// the derived class.
//
#ifndef	_movetabl_h
#define	_movetabl_h

#include "cube.h"
#include <iostream>
#include <fstream>

class MoveTable
{
public:
	// The constructor must be provided with a cube to be
	//   manipulated during table generation, the size
	//   of the table (number of entries), and whether
	//   or not the table is a phase 2 table.  If the
	//   table is a phase 2 table, then only quarter
	//   turn moves are allowed for F,B,L, and R.
	MoveTable(Cube& cube, int tableSize, int phase2=0);

	virtual ~MoveTable();

	// Initialize the pruning table by either generating it
	//   or loading it from an existing file
	virtual void Initialize(char* fileName);

	// Overloaded subscript operator allows standard C++ indexing
	//   (i.e. MoveTable[i][j]) for accessing table values.
	virtual int* operator[](int index);

	// Obtain the size of the table (number of logical entries)
	virtual int SizeOf(void) { return TableSize; }

	// Dump table contents
	virtual void Dump(void);

protected:
	// These functions must be overloaded in the derived
	//   class in order to provide the appropriate mapping
	//   between ordinal and cube state.
	virtual int OrdinalFromCubeState(void) = 0;
	virtual void OrdinalToCubeState(int ordinal) = 0;

private:
	// Generate the table
	void Generate(void);
	// Save the table to a file
	void Save(char* fileName);
	// Load the table from a file
	void Load(std::ifstream& infile);

	// Copies of important variables
	Cube& TheCube;
	// Number of entries in the pruning table
	int TableSize;
	int Phase2;
	// The table pointer
//	int (*Table)[Cube::Move::NumberOfClockwiseQuarterTurnMoves];
	int (*Table)[Cube::NumberOfClockwiseQuarterTurnMoves];	
};

#endif	// _movetabl_h
