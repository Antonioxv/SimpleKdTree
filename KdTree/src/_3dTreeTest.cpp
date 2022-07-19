/*
 * @Date: 2022-07-19 22:58:13
 * @LastEditors: fuchaoxin
 * @LastEditTime: 2022-07-20 00:23:32
 * @FilePath: \KdTree\KdTree\src\_3dTreeTest.cpp
 */
#include "kdtreepch.h"

#include "KdTree/KdTree.h"

using namespace Kdtree;
using std::vector;


const double _min = 0.0;
const double _max = 800.0;
const int maxNum = 50;


int _random(int min, int max) {
	// Returns a random integer in [min, max).
	return min + rand() % (max - min);
};

double _random() {
	// Returns a random real in [0,1).
	return rand() / (RAND_MAX + 1.0);
};

double _random(double min, double max) {
	// Returns a random real in [min,max).
	return min + (max - min) * _random();
};


int main(void) {
	
	



	return 0;
}