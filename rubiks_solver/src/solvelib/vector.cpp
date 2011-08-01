// Vector manipulation routines

#include <string.h>		// For memcpy()
#include <iostream>

// CopyVector - Clone a vector
void CopyVector(int* fromVector, int* toVector, int n)
{
	memcpy(toVector, fromVector, n*sizeof(int));
}

// PrintVector - Print the contents of the vector
void PrintVector(int* vector, int n)
{
	int i;
	for (i = 0; i < n; i++)
		std::cout << vector[i] << " ";
	std::cout << std::endl;
}
