#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	// Calculate bounds of a cell
	CellWidth  = Width  / Cols;
	CellHeight = Height / Rows;

	// Grid is centred on the origin, so bottom-left corner is at (-W/2, -H/2)
	CellOrigin = FVector2D{ -Width * 0.5f, -Height * 0.5f };

	// Create all cells row by row (bottom to top), column by column (left to right)
	Cells.reserve(Rows * Cols);
	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float left   = CellOrigin.X + col * CellWidth;
			float bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int idx = PositionToIndex(Agent.GetPosition());
	if (idx >= 0 && idx < static_cast<int>(Cells.size()))
	{
		Cells[idx].Agents.push_back(&Agent);
	}
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int oldIdx = PositionToIndex(OldPos);
	int newIdx = PositionToIndex(Agent.GetPosition());

	if (oldIdx == newIdx)
		return; // still in the same cell, nothing to do

	// Remove from old cell
	if (oldIdx >= 0 && oldIdx < static_cast<int>(Cells.size()))
	{
		Cells[oldIdx].Agents.remove(&Agent);
	}

	// Add to new cell
	if (newIdx >= 0 && newIdx < static_cast<int>(Cells.size()))
	{
		Cells[newIdx].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	// Build a query rect around the agent that represents the neighborhood circle's AABB
	FVector2D AgentPos = Agent.GetPosition();
	FRect QueryRect;
	QueryRect.Min = { AgentPos.X - QueryRadius, AgentPos.Y - QueryRadius };
	QueryRect.Max = { AgentPos.X + QueryRadius, AgentPos.Y + QueryRadius };

	for (Cell& cell : Cells)
	{
		// Skip cells that don't overlap the query rect at all
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		// Check each agent in this candidate cell
		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (!pOther || pOther == &Agent)
				continue;

			float dist = FVector2D::Distance(AgentPos, pOther->GetPosition());
			if (dist <= QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	const float Z = 100.f;

	for (const Cell& cell : Cells)
	{
		const float L = cell.BoundingBox.Min.X;
		const float R = cell.BoundingBox.Max.X;
		const float B = cell.BoundingBox.Min.Y;
		const float T = cell.BoundingBox.Max.Y;

		FVector BL(L, B, Z);
		FVector BR(R, B, Z);
		FVector TR(R, T, Z);
		FVector TL(L, T, Z);

		DrawDebugLine(pWorld, BL, BR, FColor::Blue, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, BR, TR, FColor::Blue, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, TR, TL, FColor::Blue, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, TL, BL, FColor::Blue, false, -1.f, 0, 1.f);

		// Only show agent count for occupied cells
		int count = static_cast<int>(cell.Agents.size());
		if (count > 0)
		{
			FVector Centre((L + R) * 0.5f, (B + T) * 0.5f, Z + 10.f);
			DrawDebugString(pWorld, Centre, FString::FromInt(count), nullptr,
				FColor::White, 0.f, true, 1.f);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	// Offset position relative to the grid origin
	int col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	// Clamp to grid bounds to handle agents at the very edge or slightly outside
	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}