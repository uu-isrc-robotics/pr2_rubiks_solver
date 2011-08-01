// pruningt.cpp - PruningTable class implementation

#include "pruningt.h"	// PruningTable
#include "movetabl.h"	// MoveTable
//#include <iostream>
#include <fstream>
#include <iomanip>

PruningTable::PruningTable(MoveTable& moveTable1, MoveTable& moveTable2,
	int homeOrdinal1, int homeOrdinal2)
	: MoveTable1(moveTable1),
	  MoveTable2(moveTable2),
	  HomeOrdinal1(homeOrdinal1),
	  HomeOrdinal2(homeOrdinal2)
{
	// Initialize table sizes
	MoveTable1Size = MoveTable1.SizeOf();
	MoveTable2Size = MoveTable2.SizeOf();
	TableSize = MoveTable1Size*MoveTable2Size;

	// Allocate the table
	//   round up to an int and determine
	//   the number of bytes to be allocated
	AllocationSize = ((TableSize+7)/8)*4;
	// The following seems like it should work, but	leads
	// to deallocation problems when TwstChce.ptb is
	// regenerated -- why?
	// AllocationSize = TableSize/2;
	Table = new char[AllocationSize];
}

void PruningTable::Initialize(char* fileName)
{
	std::ifstream infile(fileName, std::ifstream::in | std::ifstream::binary );//| ios::nocreate);
	if (!infile)	// If the pruning table file is absent...
	{
		// Generate the table and save it to a file
//		std::cout << "Generating" << std::endl;
		Generate();
//		std::cout << "Saving" << std::endl;
		Save(fileName);
//		std::cout << "Done Saving" << std::endl;
	}
	else		// The pruning table files exists
	{
		// Load the existing file
//		std::cout << "Loading" << std::endl;
		Load(infile);
	}
}

PruningTable::~PruningTable()
{
	// Deallocate table storage
	delete [] Table;
}

// Performs a breadth first search to fill the pruning table
void PruningTable::Generate(void)
{
	unsigned int depth = 0; // Current search depth
	int numberOfNodes;		// Number of nodes generated
	int ordinal1, ordinal2; // Table coordinates
	int index, index2;		// Table indices
	int move;
	int power;

	// Initialize all tables entries to "empty"
	for (index = 0; index < TableSize; index++)
		SetValue(index, Empty);

	// Get root coordinates of search tree
	//   and initialize to zero
	SetValue(MoveTableIndicesToPruningTableIndex(HomeOrdinal1, HomeOrdinal2),
		depth);
	numberOfNodes = 1;	// Count root node here

	// While empty table entries exist...
	while (numberOfNodes < TableSize)
	{
		// Scan all entries looking for entries
		//   corresponding to the current depth
		for (index = 0; index < TableSize; index++)
		{
			// Expand the nodes at the current depth only
			if (GetValue(index) == depth)
			{
				// Apply each possible move
				for (move = Cube::R; move <= Cube::B; move++)
				{
					PruningTableIndexToMoveTableIndices(index, ordinal1, ordinal2);
					// Apply each of the three quarter turns
					for (power = 1; power < 4; power++)
					{
						// Use the move mapping table to find the child node
						ordinal1 = MoveTable1[ordinal1][move];
						ordinal2 = MoveTable2[ordinal2][move];
						index2 = MoveTableIndicesToPruningTableIndex(
							ordinal1, ordinal2);

						// Update previously unexplored nodes only
						if (GetValue(index2) == Empty)
						{
							SetValue(index2, depth+1);
							numberOfNodes++;
						}
// An optimization that could be done, but is probably not worthwhile
//						if (phase2 && move != Cube::Move::U && move != Cube::Move::D && power == 1)
//							break;
					}
				}
			}
		}
		depth++;
//		std::cout << "Completed Depth = " << depth << std::endl;
	}
}

void PruningTable::PruningTableIndexToMoveTableIndices(
	int index, int& ordinal1, int& ordinal2)
{
	// Split the pruning table index
	ordinal1 = index/MoveTable2Size;
	ordinal2 = index%MoveTable2Size;
	return;
}

int PruningTable::MoveTableIndicesToPruningTableIndex(
	int ordinal1, int ordinal2)
{
	// Combine move table indices
	return ordinal1*MoveTable2Size+ordinal2;
}

unsigned int PruningTable::OffsetToEntryMask[2] = {
	Empty<<0,  Empty<<4 };

unsigned int PruningTable::OffsetToShiftCount[2] = {
	0, 4 };
	
unsigned int PruningTable::GetValue(int index)
{
	// Retrieve the proper nybble
	int offset = index%2;
	return (Table[index/2]&OffsetToEntryMask[offset])>>OffsetToShiftCount[offset];
}

void PruningTable::SetValue(int index, unsigned int value)
{
	// Set the proper nybble
	int i = index/2;
	int offset = index%2;
	Table[i] = (Table[i] & ~OffsetToEntryMask[offset]) |
		(value<<OffsetToShiftCount[offset]);
}

void PruningTable::Save(char* fileName)
{
	std::ofstream outfile(fileName, std::ofstream::out | std::ofstream::binary);
	outfile.write(Table, AllocationSize);
}

void PruningTable::Load(std::ifstream& infile)
{
	infile.read(Table, AllocationSize);
}

// Output the pruning table in human readable form
void PruningTable::Dump(void)
{
	int index;
	for (index = 0; index < TableSize; index++)
	{
		std::cout << std::setw(7) << index << ": "
		     << std::setw(2) << GetValue(index) << std::endl;
	}
}
