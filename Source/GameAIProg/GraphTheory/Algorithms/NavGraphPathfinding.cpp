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

	FVector2D startSnapped{};
	FVector2D endSnapped{};
	TriPolygon::Triangle const* pStartTriangle = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(startPos, startSnapped);
	TriPolygon::Triangle const* pEndTriangle   = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(endPos,   endSnapped);
 
	if (!pStartTriangle || !pEndTriangle)
		return finalPath;
 
	if (*pStartTriangle == *pEndTriangle)
	{
		finalPath.push_back(startSnapped);
		finalPath.push_back(endSnapped);
		return finalPath;
	}
 
	std::unique_ptr<NavGraph> pClonedGraph = pNavGraph->Clone();
 
	NavGraphNode* pStartNode = new NavGraphNode(startSnapped, -1);
	int startNodeId = pClonedGraph->AddNode(std::unique_ptr<Node>(pStartNode));
 
	for (auto const& edge : pStartTriangle->GetEdges())
	{
		auto edgeIdxOpt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!edgeIdxOpt.has_value()) continue;
 
		int neighborId = pClonedGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
		if (neighborId == Graphs::InvalidNodeId) continue;
 
		FVector2D neighborPos = pClonedGraph->GetNode(neighborId)->GetPosition();
		float dist = FVector2D::Distance(startSnapped, neighborPos);
 
		auto pConn = std::make_unique<Connection>(startNodeId, neighborId);
		pConn->SetWeight(dist);
		pClonedGraph->AddConnection(std::move(pConn));
	}
 
	NavGraphNode* pEndNode = new NavGraphNode(endSnapped, -1);
	int endNodeId = pClonedGraph->AddNode(std::unique_ptr<Node>(pEndNode));
 
	for (auto const& edge : pEndTriangle->GetEdges())
	{
		auto edgeIdxOpt = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!edgeIdxOpt.has_value()) continue;
 
		int neighborId = pClonedGraph->GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
		if (neighborId == Graphs::InvalidNodeId) continue;
 
		FVector2D neighborPos = pClonedGraph->GetNode(neighborId)->GetPosition();
		float dist = FVector2D::Distance(endSnapped, neighborPos);
 
		auto pConn = std::make_unique<Connection>(endNodeId, neighborId);
		pConn->SetWeight(dist);
		pClonedGraph->AddConnection(std::move(pConn));
	}
 
	AStar aStar(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	std::vector<Node*> nodePath = aStar.FindPath(
		pClonedGraph->GetNode(startNodeId).get(),
		pClonedGraph->GetNode(endNodeId).get()
	);
 
	if (nodePath.empty())
		return finalPath;
 
	for (Node* pNode : nodePath)
		debugNodePositions.push_back(pNode->GetPosition());
 
	for (Node* pNode : nodePath)
		finalPath.push_back(pNode->GetPosition());
 
	// Extra: Run SSFA path smoother on the node path
	// Uncomment once CreateNavigationGraph and basic pathfinding are verified working:
	// debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
 
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}