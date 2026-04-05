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
		// Returns the signed area of triangle (o->a, o->b).
		// Positive = CCW in standard math = b is to the LEFT of o->a
		// In Unreal (Y down / left-handed): positive = b is to the RIGHT of o->a
		static float TriArea2(FVector2D const& o, FVector2D const& a, FVector2D const& b)
		{
			return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
		}

		// Build a list of portals from the A* node path.
		// Each portal is the shared edge between two consecutive triangles in the path.
		// P1 = right vertex, P2 = left vertex (relative to travel direction, in Unreal Y-down space).
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals{};

			// Collect raw edge data for each middle node (skip start and end, they have EdgeIdx=-1)
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

			// Orient first portal: standing at start, looking toward the node after the portal.
			// TriArea2(start, next, p1) > 0 means p1 is to the RIGHT in Unreal (Y-down).
			// We want P1=right, P2=left.
			{
				FVector2D start = Path[0]->GetPosition();
				FVector2D next  = Path[2]->GetPosition();
				FVector2D p1 = Raw[0].A;
				FVector2D p2 = Raw[0].B;
				if (TriArea2(start, next, p1) <= 0.f)
					std::swap(p1, p2);
				Portals.push_back({ p1, p2 });
			}

			// Orient each subsequent portal relative to the previous one.
			// Consecutive portals always share exactly one vertex (the triangle corner they pivot around).
			// That shared vertex must stay on the same side across portals.
			for (int i = 1; i < (int)Raw.size(); ++i)
			{
				FVector2D prevRight = Portals.back().P1;
				FVector2D prevLeft  = Portals.back().P2;
				FVector2D p1 = Raw[i].A;
				FVector2D p2 = Raw[i].B;

				if (p1 == prevLeft || p2 == prevRight)
					std::swap(p1, p2);

				Portals.push_back({ p1, p2 });
			}

			// Degenerate closing portal at destination
			FVector2D end = Path.back()->GetPosition();
			Portals.push_back({ end, end });

			return Portals;
		}

		// Run the funnel algorithm over the portal list.
		// Portals[0] must be a degenerate start portal { startPos, startPos } (prepended by caller).
		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			int n = (int)Portals.size();
			if (n < 2) { if (n == 1) Path.push_back(Portals[0].P1); return Path; }

			// apex = current path vertex we are smoothing from
			// rightLeg/leftLeg = vectors from apex to the right/left edges of the current funnel
			FVector2D apex     = Portals[0].P1;
			FVector2D rightLeg = Portals[1].P1 - apex;
			FVector2D leftLeg  = Portals[1].P2  - apex;
			int rightIdx = 1;
			int leftIdx  = 1;

			Path.push_back(apex);

			// cross2D(a, b): positive = b is CCW from a (standard), negative = CW
			// In Unreal Y-down: positive = b is to the RIGHT of a
			auto cross2D = [](FVector2D const& a, FVector2D const& b) -> float
			{
				return a.X * b.Y - a.Y * b.X;
			};

			for (int i = 2; i < n; ++i)
			{
				FVector2D newRight = Portals[i].P1 - apex;
				FVector2D newLeft  = Portals[i].P2  - apex;

				// --- Tighten right leg ---
				// cross2D(rightLeg, newRight) < 0 means newRight is CW from rightLeg
				// In Unreal Y-down: CW = moving to the right = tightening the funnel on the right side
				if (cross2D(rightLeg, newRight) < 0.f)
				{
					// Check if newRight has crossed over the left leg
					// cross2D(leftLeg, newRight) < 0 means newRight is to the right of leftLeg = crossed
					if (cross2D(leftLeg, newRight) < 0.f)
					{
						// New right is past the left leg: left leg vertex is a corner
						apex = apex + leftLeg;
						Path.push_back(apex);
						// Restart funnel from the left leg portal
						int restart  = leftIdx;
						rightIdx     = restart;
						leftIdx      = restart;
						rightLeg     = Portals[restart].P1 - apex;
						leftLeg      = Portals[restart].P2  - apex;
						i            = restart; // for-loop ++i will move to restart+1
						continue;
					}
					else
					{
						rightLeg = newRight;
						rightIdx = i;
					}
				}

				// --- Tighten left leg ---
				// cross2D(leftLeg, newLeft) > 0 means newLeft is CCW from leftLeg
				// In Unreal Y-down: CCW = moving to the left = tightening the funnel on the left side
				if (cross2D(leftLeg, newLeft) > 0.f)
				{
					// Check if newLeft has crossed over the right leg
					if (cross2D(rightLeg, newLeft) > 0.f)
					{
						// New left is past the right leg: right leg vertex is a corner
						apex = apex + rightLeg;
						Path.push_back(apex);
						// Restart funnel from the right leg portal
						int restart  = rightIdx;
						rightIdx     = restart;
						leftIdx      = restart;
						rightLeg     = Portals[restart].P1 - apex;
						leftLeg      = Portals[restart].P2  - apex;
						i            = restart; // for-loop ++i will move to restart+1
						continue;
					}
					else
					{
						leftLeg = newLeft;
						leftIdx = i;
					}
				}
			}

			// Add destination (last portal is degenerate: P1==P2==endPos)
			Path.push_back(Portals.back().P1);
			return Path;
		}

	private:
		SSFA() {}
		~SSFA() {}
	};
}
