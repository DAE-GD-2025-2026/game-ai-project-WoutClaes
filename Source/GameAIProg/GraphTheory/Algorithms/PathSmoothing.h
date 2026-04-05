#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		static float TriArea2(FVector2D const& o, FVector2D const& a, FVector2D const& b)
		{
			return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
		}

		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals{};

			struct RawPortal { FVector2D A, B; };
			std::vector<RawPortal> Raw{};

			for (int i = 1; i < (int)Path.size() - 1; ++i)
			{
				NavGraphNode const* pNode = dynamic_cast<NavGraphNode const*>(Path[i]);
				if (!pNode || pNode->GetEdgeIdx() < 0) continue;
				auto const& edge = NavPoly.GetEdges()[pNode->GetEdgeIdx()];
				Raw.push_back({ FVector2D{edge.GetP1(NavPoly)}, FVector2D{edge.GetP2(NavPoly)} });
			}

			if (Raw.empty())
			{
				Portals.push_back({ Path.back()->GetPosition(), Path.back()->GetPosition() });
				return Portals;
			}

			{
				FVector2D start = Path[0]->GetPosition();
				FVector2D next  = Path[2]->GetPosition();
				FVector2D p1 = Raw[0].A, p2 = Raw[0].B;
				if (TriArea2(start, next, p1) <= 0.f) std::swap(p1, p2);
				Portals.push_back({ p1, p2 });
			}

			for (int i = 1; i < (int)Raw.size(); ++i)
			{
				FVector2D prevRight = Portals.back().P1;
				FVector2D prevLeft  = Portals.back().P2;
				FVector2D p1 = Raw[i].A, p2 = Raw[i].B;
				if (p1 == prevLeft || p2 == prevRight) std::swap(p1, p2);
				Portals.push_back({ p1, p2 });
			}

			Portals.push_back({ Path.back()->GetPosition(), Path.back()->GetPosition() });
			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			int n = (int)Portals.size();
			if (n < 2) { if (n == 1) Path.push_back(Portals[0].P1); return Path; }

			auto cross2D = [](FVector2D const& a, FVector2D const& b) -> float
			{
				return a.X * b.Y - a.Y * b.X;
			};

			FVector2D apex     = Portals[0].P1;
			FVector2D rightLeg = Portals[1].P1 - apex;
			FVector2D leftLeg  = Portals[1].P2  - apex;
			int rightIdx = 1;
			int leftIdx  = 1;

			Path.push_back(apex);

			for (int i = 2; i < n; ++i)
			{
				FVector2D newRight = Portals[i].P1 - apex;
				FVector2D newLeft  = Portals[i].P2  - apex;

				if (cross2D(rightLeg, newRight) < 0.f)
				{
					if (cross2D(leftLeg, newRight) < 0.f)
					{
						apex = apex + leftLeg;
						Path.push_back(apex);

						int restart  = leftIdx + 1;
						if (restart >= n) break;

						rightIdx = restart;
						leftIdx  = restart;
						rightLeg = Portals[restart].P1 - apex;
						leftLeg  = Portals[restart].P2  - apex;
						i = restart;
						continue;
					}
					rightLeg = newRight;
					rightIdx = i;
				}

				if (cross2D(leftLeg, newLeft) > 0.f)
				{
					if (cross2D(rightLeg, newLeft) > 0.f)
					{
						apex = apex + rightLeg;
						Path.push_back(apex);

						int restart  = rightIdx + 1;
						if (restart >= n) break;

						rightIdx = restart;
						leftIdx  = restart;
						rightLeg = Portals[restart].P1 - apex;
						leftLeg  = Portals[restart].P2  - apex;
						i = restart;
						continue;
					}
					leftLeg = newLeft;
					leftIdx = i;
				}
			}

			Path.push_back(Portals.back().P1);
			return Path;
		}

	private:
		SSFA() {}
		~SSFA() {}
	};
}
