#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	
	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startRecord);
 
	NodeRecord currentRecord{};
	
	while (!openList.empty())
	{
		currentRecord = *std::min_element(openList.begin(), openList.end());
 
		if (currentRecord.pNode == pGoalNode)
			break;
 
		for (Connection* pConnection : pGraph->FindConnectionsFrom(currentRecord.pNode->GetId()))
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			float newCostSoFar = currentRecord.costSoFar + pConnection->GetWeight();
 
			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });
 
			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= newCostSoFar)
					continue;
 
				closedList.erase(closedIt);
			}
 
			auto openIt = std::find_if(openList.begin(), openList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });
 
			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= newCostSoFar)
					continue;
 
				openList.erase(openIt);
			}
 
			NodeRecord newRecord{};
			newRecord.pNode = pNextNode;
			newRecord.pConnection = pConnection;
			newRecord.costSoFar = newCostSoFar;
			newRecord.estimatedTotalCost = newCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(newRecord);
		}
 
		openList.erase(std::remove(openList.begin(), openList.end(), currentRecord), openList.end());
		closedList.push_back(currentRecord);
	}
 
	if (currentRecord.pNode != pGoalNode)
		return path;
 
	while (currentRecord.pNode != pStartNode)
	{
		path.push_back(currentRecord.pNode);
 
		Node* pFrom = pGraph->GetNode(currentRecord.pConnection->GetFromId()).get();
 
		auto it = std::find_if(closedList.begin(), closedList.end(),
			[pFrom](const NodeRecord& r) { return r.pNode == pFrom; });
 
		currentRecord = *it;
	}
 
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}