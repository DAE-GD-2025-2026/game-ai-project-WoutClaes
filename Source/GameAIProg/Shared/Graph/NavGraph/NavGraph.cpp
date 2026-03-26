#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	auto const& edges     = pNavPoly->GetEdges();
	auto const& triangles = pNavPoly->GetTriangles();
 
	for (int edgeIdx = 0; edgeIdx < static_cast<int>(edges.size()); ++edgeIdx)
	{
		int sharedCount = 0;
		for (auto const& tri : triangles)
		{
			if (tri.HasEdge(edges[edgeIdx]))
				++sharedCount;
		}
 
		if (sharedCount >= 2)
		{
			FVector p1 = edges[edgeIdx].GetP1(*pNavPoly);
			FVector p2 = edges[edgeIdx].GetP2(*pNavPoly);
			FVector2D midpoint{ (p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f };
 
			AddNode(std::make_unique<NavGraphNode>(midpoint, edgeIdx));
		}
	}
	
	for (auto const& tri : triangles)
	{
		auto triEdges = tri.GetEdges();
 
		std::vector<int> nodeIds{};
		for (auto const& edge : triEdges)
		{
			auto edgeIdxOpt = pNavPoly->FindEdgeIndex(edge);
			if (!edgeIdxOpt.has_value()) continue;
 
			int nodeId = GetNodeIdFromEdgeIndex(edgeIdxOpt.value());
			if (nodeId != Graphs::InvalidNodeId)
				nodeIds.push_back(nodeId);
		}
 
		for (int i = 0; i < static_cast<int>(nodeIds.size()); ++i)
		{
			for (int j = i + 1; j < static_cast<int>(nodeIds.size()); ++j)
			{
				FVector2D posA = GetNode(nodeIds[i])->GetPosition();
				FVector2D posB = GetNode(nodeIds[j])->GetPosition();
				float dist = FVector2D::Distance(posA, posB);
 
				// Graph is undirected — AddConnection automatically adds the inverse
				auto pConn = std::make_unique<Connection>(nodeIds[i], nodeIds[j]);
				pConn->SetWeight(dist);
				AddConnection(std::move(pConn));
			}
		}
	}
}
