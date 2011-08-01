// movetabl.cpp - MoveTable class implementation

#include "movetabl.h"	// MoveTable
#include <iostream>
#include <fstream>
#include <iomanip>


MoveTable::MoveTable(Cube& cube, int tableSize, int phase2)
	:TheCube(cube),
	 TableSize(tableSize),
	 Phase2(phase2)
{
	// Allocate table storage
	Table = (int (*)[Cube::NumberOfClockwiseQuarterTurnMoves]) // the cast
	        (new int[TableSize*Cube::NumberOfClockwiseQuarterTurnMoves]); // the allocation
}

MoveTable::~MoveTable()
{
	// Deallocate table storage
	delete [] Table;
}

void MoveTable::Initialize(char* fileName)
{
	std::ifstream infile(fileName, std::iostream::in | std::ifstream::binary );//| std::ifstream::nocreate);
	if (!infile)	// If the move mapping table file is absent...
	{
		// Generate the table and save it to a file
//		std::cout << "Generating" << std::endl;
		Generate();
//		std::cout << "Saving" << std::endl;
		Save(fileName);
//		std::cout << "Done Saving" << std::endl;
	}
	else	// The move mapping table file exists
	{
		// Load the existing file
//		std::cout << "Loading" << std::endl;
		Load(infile);
	}
}

// Generate the table
void MoveTable::Generate(void)
{
	int ordinal;
	int move, move2;

	// Insure the cubies are in their proper slice
	TheCube.BackToHome();

	// Initialize each table entry
	for (ordinal = 0; ordinal < TableSize; ordinal++)
	{
		// Establish the proper cube state for the current ordinal
		OrdinalToCubeState(ordinal);

		// Initialize the possible moves for each entry
		for (move = Cube::R; move <= Cube::B; move++)
		{
			// Apply this move

			// Phase 1 is the group spanned by <U,D,R,L,F,B>
			// Phase 2 is the group spanned by <U,D,R2,L2,F2,B2>
			move2 = move;
			if (Phase2 && move != Cube::U && move != Cube::D)
				move2 = Cube::QuarterTurnToHalfTurnMove(move);

			TheCube.ApplyMove(move2);

			// Compute a new ordinal from the new cube state
			Table[ordinal][move] = OrdinalFromCubeState();
			// Unapply this move
			TheCube.ApplyMove(Cube::InverseOfMove(move2));
		}
	}
}

int* MoveTable::operator[](int index)
{
	return Table[index];
}

void MoveTable::Save(char* fileName)
{
	std::ofstream outfile(fileName, std::ofstream::out | std::ofstream::binary);
	int index;
	for (index = 0; index < TableSize; index++)
		outfile.write((char*)&Table[index],
			Cube::NumberOfClockwiseQuarterTurnMoves*sizeof(int));
}

void MoveTable::Load(std::ifstream& infile)
{
	int index;
	for (index = 0; index < TableSize; index++)
		infile.read((char*)&Table[index],
			Cube::NumberOfClockwiseQuarterTurnMoves*sizeof(int));
}

// Output the move table in human readable form
void MoveTable::Dump(void)
{
	int ordinal;
	int move, move2;
	// For each table entry...
	for (ordinal = 0; ordinal < TableSize; ordinal++)
	{
		std::cout << std::setw(8) << ordinal << ": ";
		// For each possible move...
		for (move = Cube::R; move <= Cube::B; move++)
		{
			move2 = move;
			if (Phase2)
			{
				if (move != Cube::U && move != Cube::D)
					move2 = Cube::QuarterTurnToHalfTurnMove(move);
			}
			std::cout << Cube::NameOfMove(move2) << ":"
			     << std::setw(8) << Table[ordinal][move] << " ";
		}
		std::cout << std::endl;
	}
}
