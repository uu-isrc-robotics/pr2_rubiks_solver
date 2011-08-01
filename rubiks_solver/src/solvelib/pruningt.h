// pruningt.h - PruningTable class definition
//
// Constructs a pruning table from a pair of move mapping
// tables.  The pruning table contains an entry corresponding
// to each possible pairing of entries from the two tables.
// Thus the number of pruning table entries is the product of
// the number of entries of the two move mapping tables.  A
// breadth first search is performed until the table is filled.
// Each table entry contains the number of moves away from
// the cube home configuration that is required to reach that
// particular configuration.  Since a breadth first search is
// performed, the distances are minimal and therefore they
// constitute an admissible heuristic.  When an admissible
// heuristic is used to prune a heuristic search such as
// IDA*, the search is guaranteed to find an optimal (i.e.
// least number of moves possible) solution.
//
#ifndef	_pruningt_h
#define	_pruningt_h

//#include <iostream>
#include <fstream>

class MoveTable;


class PruningTable
{
public:
	// Constructor - Must provide a pair of move mapping tables
	//   and the associated ordinal corresponding to the cube's
	//   "home" configuration.  The home ordinals correspond to
	//   the root node of the search.
	PruningTable(MoveTable& moveTable1, MoveTable& moveTable2,
		int homeOrdinal1, int homeOrdinal2);

	~PruningTable();

	// Initialize the pruning table by either generating it
	//   or loading it from an existing file
	void Initialize(char* fileName);

	// Convert a pruning table index to the associated pair
	//   of move mapping table indices
	void PruningTableIndexToMoveTableIndices(int index, int& ordinal1, int& ordinal2);

	// Convert a pair of move mapping table indices to the
	//   associated pruning table index
	int MoveTableIndicesToPruningTableIndex(int ordinal1, int ordinal2);

	// Get a pruning table value corresponding to the specified index
	unsigned int GetValue(int index);

	// Set a pruning table value at the specified index
	void SetValue(int index, unsigned int value);

	// Obtain the size of the table (number of logical entries)
	int SizeOf(void) { return TableSize; }

	// Dump table contents
	void Dump(void);

private:
	enum { Empty = 0x0f };	// Empty table entry

	// Generate the table using breath first search
	void Generate(void);
	// Save the table to a file
	void Save(char* fileName);
	// Load the table from a file
	void Load(std::ifstream& infile);
	
	// Copies of important variables
	MoveTable& MoveTable1;
	MoveTable& MoveTable2;
	int HomeOrdinal1;
	int HomeOrdinal2;
	int MoveTable1Size;
	int MoveTable2Size;

	// Number of entries in the pruning table
	int TableSize;
	// Actual size, in bytes, allocated for the table
	int AllocationSize;
	// The table pointer
	char (*Table);

	// Tables for dealing with nybble packing/unpacking
	static unsigned int OffsetToEntryMask[2];
	static unsigned int OffsetToShiftCount[2];
};

#endif	// _pruningt_h
