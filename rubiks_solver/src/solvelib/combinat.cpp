#ifndef	_combinat_h
#define	_combinat_h

#include "combinat.h"
#include "vector.h"	// Vector manipulation routines

// N Choose M - Compute the number of ways a subset of N items can
//   be selected from a set of M items.  N must be greater than M.
//   The formula for N choose M is:
//     N! / (M! * (N-M)!)
//
//   The N! / (N-M)! portion can be computed iteratively as:
//     N * (N-1) * (N-2) * (N-3) * ... * (N-M+1)
//
//   The M! portion can be divided out iteratively as:
//     1, 2, 3 ... M
//   (i.e. first divide partial result by 1, then by 2, then by 3,...)
//
//   Note that both require M iterations allowing the entire
//   calculation to be performed within a single loop.  Note also
//   that we want to keep the numerator as large as possible during
//   this calculation to avoid truncation error during the division.
//   This is accomplished by performing the multiplication in
//   descending order and the division in ascending order.
//
//   Since N choose M is equivalent to N choose (N-M) we can take
//   advantage of this property to optimize the calculation in
//   cases where M > N/2 since using the equivalent form results
//   in a smaller M recalling that M is the number of loop iterations.
//
//   Note, the performance of this function could be improved by
//   using a table of precomputed results (i.e. NChooseM[N][M]).
//
int NChooseM(int N, int M)
{
	int NoverMfact = N;	// Iterates from N down to M+1 to
				//   compute N! / (N-M)!
	int Mfact = 1;		// Iterates from 1 to M to divide
				//   out the M! term
	int Result = 1;		// Holds the result of N choose M
	if (N < M) return 0;	// M must be a subset of M
	if (M > N/2) M = N-M;	// Optimization
	while (NoverMfact > M)
	{
		Result *= NoverMfact--;	// Work on the N! / (N-M)! part
		Result /= Mfact++;	// Divide out the M! part
	}
	return Result;
}

// The following pair of permutation algorithms are based on a
// description in Knuth's "Fundamental Algorithms Volume 2:
// Seminumerical Algorithms" p64.

// PermutationToOrdinal - Given a permutation contained in a
// vector of length n, compute a unique ordinal in the range
// (0,...n!-1).
//
// This algorithm is based on the notion of a "factorial number
// system".  An ordinal can be thought of as a number with a
// variable base, the sum of factorial terms, where each term
// is the product of a factorial and an associated coefficent.
// It generates the coefficients by finding the position of the
// largest, second largest, third largest, etc. elements in
// the permutation vector.  Each time it finds the ith largest
// element, it exchanges that with the element at location i.
// Thus there are i possibilities for the position of the ith
// largest element. This process yields the i coefficients.
//
// Note: The length of the vector is currently limited to
// 12 elements.
//
int PermutationToOrdinal(int* vector, int n)
{
	int Ordinal = 0;
	int Vector[12];		// Limits n <= 12
	int Limit;
	int i;
	int Coeff_i = 0;
	int Temp;

	// Make a copy of the permutation vector
	CopyVector(vector, Vector, n);
	
	for (Limit = n-1; Limit > 0; Limit--)
	{
		// Find the maximum up to the current limit
		Temp = -1;
		for (i = 0; i <= Limit; i++)
		{
			if (Vector[i] > Temp)
			{
				Temp = Vector[i];
				Coeff_i = i;
			}
		}
		// Accumulate result
		Ordinal = Ordinal*(Limit+1)+Coeff_i;

		// Exchange elements
		Temp            = Vector[Limit];
		Vector[Limit]   = Vector[Coeff_i];
		Vector[Coeff_i] = Temp;
	}
	return Ordinal;
}

// OrdinalToPermutation - Given an ordinal in the range
// (0,...n!-1) compute a unique permutation of n items.
//
// This algorithm is essentially the above algorithm run
// backwards.  It uses modulo arithmetic and division to
// produce the coefficients that drive the exchanges.
//
void OrdinalToPermutation(int Ordinal, int* vector, int n, int offset)
{
	int i;
	int Coeff_i;
	int Temp;
        // Construct an inital permutation
	for (i = 0; i < n; i++)
		vector[i] = i+offset;

	for (i = 1; i < n; i++)
	{
                // Compute the coefficent
		Coeff_i   = Ordinal % (i+1);
                // Divide out current "factorial number base"
		Ordinal   /= (i+1);

		// Exchange elements
		Temp            = vector[i];
		vector[i]       = vector[Coeff_i];
		vector[Coeff_i] = Temp;
	}
}
#endif	// _combinat_h
