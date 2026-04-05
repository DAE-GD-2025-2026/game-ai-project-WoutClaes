#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals)
{
	std::vector<FVector2D> finalPath{};

	// --- Step A: find which triangles contain start and end ---
	FVector2D startSnapped{};
	FVector2D endSnapped{};
	TriPolygon::Triangle const* pStartTri = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(startPos, startSnapped);
	TriPolygon::Triangle const* pEndTri   = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(endPos,   endSnapped);

	if (!pStartTri || !pEndTri)
		return finalPath;

	// Same triangle: straight line
	if (*pStartTri == *pEndTri)
	{
		finalPath.push_back(startSnapped);
		finalPath.push_back(endSnapped);
		return finalPath;
	}

	// --- Step B: clone graph and add temporary start/end nodes ---
	std::unique_ptr<NavGraph> pGraph = pNavGraph->Clone();

	// Add start node (EdgeIdx = -1, not on any edge)
	int startId = pGraph->AddNode(std::make_unique<NavGraphNode>(startSnapped, -1));
	for (auto const& edge : pStartTri->GetEdges())
	{
		auto opt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!opt) continue;
		int nId = pGraph->GetNodeIdFromEdgeIndex(*opt);
		if (nId == Graphs::InvalidNodeId) continue;
		float dist = FVector2D::Distance(startSnapped, pGraph->GetNode(nId)->GetPosition());
		auto c = std::make_unique<Connection>(startId, nId);
		c->SetWeight(dist);
		pGraph->AddConnection(std::move(c));
	}

	// Add end node (EdgeIdx = -1)
	int endId = pGraph->AddNode(std::make_unique<NavGraphNode>(endSnapped, -1));
	for (auto const& edge : pEndTri->GetEdges())
	{
		auto opt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!opt) continue;
		int nId = pGraph->GetNodeIdFromEdgeIndex(*opt);
		if (nId == Graphs::InvalidNodeId) continue;
		float dist = FVector2D::Distance(endSnapped, pGraph->GetNode(nId)->GetPosition());
		auto c = std::make_unique<Connection>(endId, nId);
		c->SetWeight(dist);
		pGraph->AddConnection(std::move(c));
	}

	// --- Step C: run A* ---
	AStar aStar(pGraph.get(), HeuristicFunctions::Euclidean);
	std::vector<Node*> nodePath = aStar.FindPath(
		pGraph->GetNode(startId).get(),
		pGraph->GetNode(endId).get()
	);

	if (nodePath.empty())
		return finalPath;

	// Store raw A* positions for debug rendering
	for (Node* pNode : nodePath)
		debugNodePositions.push_back(pNode->GetPosition());

	// Direct connection, no portals needed
	if (nodePath.size() <= 2)
	{
		for (Node* pNode : nodePath)
			finalPath.push_back(pNode->GetPosition());
		return finalPath;
	}

	// --- Step D: smooth path with SSFA ---
	// FindPortals builds the oriented portal list (no degenerate start)
	std::vector<NavLine> rawPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	debugPortals = rawPortals;

	// Prepend degenerate start portal so OptimizePortals gets the correct apex
	std::vector<NavLine> allPortals{};
	allPortals.push_back({ startSnapped, startSnapped });
	allPortals.insert(allPortals.end(), rawPortals.begin(), rawPortals.end());

	finalPath = SSFA::OptimizePortals(allPortals, *pNavGraph->GetNavPolygon());

	// Fallback to raw A* path if SSFA failed
	if (finalPath.size() < 2)
	{
		finalPath.clear();
		for (Node* pNode : nodePath)
			finalPath.push_back(pNode->GetPosition());
	}

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine>   debugPortals{};
	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}
