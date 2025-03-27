#include <iostream>
#include "Hungarian.h"


int main(void)
{
    // // please use "-std=c++11" for this initialization of vector.
	// vector< vector<double> > costMatrix = { { 10, 19, 8, 15, 0 }, 
	// 									  { 10, 18, 7, 17, 0 }, 
	// 									  { 13, 16, 9, 14, 0 }, 
	// 									  { 12, 19, 8, 18, 0 } };

	// vector< vector<double> > costMatrix = { { 40, 60, 15 }, 
	// 									  { 25, 30, 45 }, 
	// 									  { 55, 30, 25 }};

	vector< vector<double> > costMatrix = { { 3, 2, 1, 0, 3, 7, 10, 12, 13, 14 }, 
										{ 9, 8, 7, 4, 1, 1, 4, 6, 7, 8 }, 
										{ 6, 5, 4, 1, 0, 4, 7, 9, 10, 11 }, 
										{ 12, 11, 10, 7, 4, 0, 1, 3, 4, 5 }, 
										{ 15, 14, 13, 10, 7, 3, 0, 0, 1, 2 }, 
										{ 2, 1, 0, 1, 4, 8, 11, 13, 14, 15 }, 
										{ 8, 7, 6, 3, 0, 2, 5, 7, 8, 9 }, 
										{ 5, 4, 3, 0, 1, 5, 8, 10, 11, 12 }, 
										{ 10, 9, 8, 5, 2, 0, 3, 5, 6, 7 }, 
										{ 13, 12, 11, 8, 5, 1, 0, 2, 3, 4 } };

	HungarianAlgorithm HungAlgo;
	vector<int> assignment;

	double cost = HungAlgo.Solve(costMatrix, assignment);

	for (unsigned int x = 0; x < costMatrix.size(); x++)
		std::cout << (x+1) << "," << (assignment[x]+1) << "\n";

	std::cout << "\ncost: " << cost << std::endl;

	return 0;
}
