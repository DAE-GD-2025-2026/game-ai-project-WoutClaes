#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	
	std::queue<Node*> openList;
	
	std::map<Node*, Node*> closedList;
	
	openList.push(pStartNode);
	closedList[pStartNode] = nullptr;
	
	bool found = false;
 
	while (!openList.empty())
	{
		Node* pCurrent = openList.front();
		openList.pop();
 
		if (pCurrent == pDestinationNode)
		{
			found = true;
			break;
		}
		
		for (Connection* pConnection : pGraph->FindConnectionsFrom(pCurrent->GetId()))
		{
			Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();
			
			if (closedList.find(pNeighbor) == closedList.end())
			{
				closedList[pNeighbor] = pCurrent;
				openList.push(pNeighbor);
			}
		}
	}
	
	if (!found)
		return path;
	
	Node* pCurrent = pDestinationNode;
	while (pCurrent != nullptr)
	{
		path.push_back(pCurrent);
		pCurrent = closedList[pCurrent];
	}
 
	std::reverse(path.begin(), path.end());
	return path;
}
