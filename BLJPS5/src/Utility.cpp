#include "Node.h"
float pathLength(std::vector<Coordinate> & solutionPath)
{
	if (solutionPath.size() == 0)
		return 0;
	float dist = 0;
	for (int i = 0; i<solutionPath.size() - 1; i++)
		dist += solutionPath[i].distSqrt(solutionPath[i + 1]);
	return dist;
}
bool nonSimilarFloat(float l, float r)
{
	return abs(1.0f - l / r) > 0.01;
}