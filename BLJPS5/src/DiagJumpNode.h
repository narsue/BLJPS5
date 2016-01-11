#pragma once
#include <vector>

struct DiagJumpNode
{
	DiagJumpNode()
	{
	}
	DiagJumpNode(short _sI)
	{
		sI = _sI;
		dI = -1;
	}
	short sI, dI;
	vector<short> jumps;
};