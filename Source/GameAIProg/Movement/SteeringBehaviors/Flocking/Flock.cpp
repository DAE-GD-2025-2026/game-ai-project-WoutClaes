#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

	// --- Spawn agents at random positions within the world ---
	for (int i = 0; i < FlockSize; ++i)
	{
		float RandX = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		float RandY = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		FVector SpawnLoc(RandX, RandY, 90.f);

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLoc,
			FRotator::ZeroRotator, SpawnParams);

		// Agents must NOT tick on their own — the Flock drives their ticks
		if (Agents[i])
			Agents[i]->SetActorTickEnabled(false);
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// --- Record initial positions for cell tracking ---
	OldPositions.SetNum(FlockSize);
	for (int i = 0; i < FlockSize; ++i)
		OldPositions[i] = Agents[i] ? Agents[i]->GetPosition() : FVector2D::ZeroVector;

	// --- Build the spatial partition ---
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		WorldSize, WorldSize,
		NrOfCellsX, NrOfCellsX,
		FlockSize);

	for (ASteeringAgent* pAgent : Agents)
		if (pAgent) pPartitionedSpace->AddAgent(*pAgent);
#else
	// --- Memory pool for brute-force neighbours ---
	Neighbors.SetNum(FlockSize);
#endif

	// --- Create steering behaviors ---
	pCohesionBehavior   = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior   = std::make_unique<VelocityMatch>(this);
	pSeekBehavior       = std::make_unique<Seek>();
	pWanderBehavior     = std::make_unique<Wander>();
	pEvadeBehavior      = std::make_unique<Evade>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>
		{
			{ pCohesionBehavior.get(),   0.2f },
			{ pSeparationBehavior.get(), 0.4f },
			{ pVelMatchBehavior.get(),   0.3f },
			{ pSeekBehavior.get(),       0.5f },
			{ pWanderBehavior.get(),     0.2f },
		}
	);

	std::vector<ISteeringBehavior*> priorityBehaviors;
	if (pAgentToEvade)
		priorityBehaviors.push_back(pEvadeBehavior.get());
	priorityBehaviors.push_back(pBlendedSteering.get());

	pPrioritySteering = std::make_unique<PrioritySteering>(priorityBehaviors);

	for (ASteeringAgent* pAgent : Agents)
		if (pAgent) pAgent->SetSteeringBehavior(pPrioritySteering.get());
}

Flock::~Flock()
{
	for (ASteeringAgent* pAgent : Agents)
	{
		if (IsValid(pAgent))
			pAgent->Destroy();
	}
	Agents.Empty();
}

void Flock::Tick(float DeltaTime)
{
	// Update evade target each frame
	if (pAgentToEvade)
	{
		FSteeringParams evadeTarget{};
		evadeTarget.Position       = pAgentToEvade->GetPosition();
		evadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(evadeTarget);
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		if (!pAgent) continue;

		pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);

		// Render before tick so circle, box and yellow spheres are all in sync
		if (DebugRenderNeighborhood && i == 0)
			RenderNeighborhood();

		pAgent->Tick(DeltaTime);

		OldPositions[i] = pAgent->GetPosition();
	}

	if (DebugRenderPartitions)
		pPartitionedSpace->RenderCells();
#else
	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		if (!pAgent) continue;

		RegisterNeighbors(pAgent);

		// Render before tick so circle and yellow spheres are all in sync
		if (DebugRenderNeighborhood && i == 0)
			RenderNeighborhood();

		pAgent->Tick(DeltaTime);
	}
#endif
}

void Flock::RenderDebug()
{
	if (!DebugRenderSteering) 
		return;

	for (ASteeringAgent* pAgent : Agents)
		if (pAgent) 
			pAgent->SetDebugRenderingEnabled(true);
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive,
			ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		// --- Debug toggles ---
		if (ImGui::Checkbox("Debug Steering", &DebugRenderSteering))
		{
			bool bEnable = DebugRenderSteering;
			for (ASteeringAgent* pAgent : Agents)
				if (pAgent) pAgent->SetDebugRenderingEnabled(bEnable);
		}
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		ImGui::Checkbox("Debug Partitions",   &DebugRenderPartitions);
#endif

		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		float w;
		w = Behaviors[0].Weight;
		if (ImGui::SliderFloat("Cohesion",   &w, 0.f, 1.f, "%.2f")) Behaviors[0].Weight = w;
		w = Behaviors[1].Weight;
		if (ImGui::SliderFloat("Separation", &w, 0.f, 1.f, "%.2f")) Behaviors[1].Weight = w;
		w = Behaviors[2].Weight;
		if (ImGui::SliderFloat("Alignment",  &w, 0.f, 1.f, "%.2f")) Behaviors[2].Weight = w;
		w = Behaviors[3].Weight;
		if (ImGui::SliderFloat("Seek",       &w, 0.f, 1.f, "%.2f")) Behaviors[3].Weight = w;
		w = Behaviors[4].Weight;
		if (ImGui::SliderFloat("Wander",     &w, 0.f, 1.f, "%.2f")) Behaviors[4].Weight = w;

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0 || !Agents[0]) return;

	ASteeringAgent* pFirst = Agents[0];
	FVector FirstPos3D = pFirst->GetActorLocation();

	// White circle: neighborhood radius
	DrawDebugCircle(pWorld, FirstPos3D, NeighborhoodRadius, 32,
		FColor::White, false, -1.f, 0, 3.f,
		FVector(0,1,0), FVector(1,0,0));

	// Yellow spheres on each neighbour of the first agent
	int count = GetNrOfNeighbors();
	const TArray<ASteeringAgent*>& currentNeighbors = GetNeighbors();
	for (int i = 0; i < count; ++i)
	{
		if (currentNeighbors[i])
		{
			DrawDebugSphere(pWorld, currentNeighbors[i]->GetActorLocation(),
				30.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
		}
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// Green box: the query bounding box used for cell lookup
	FVector2D pos = pFirst->GetPosition();
	FVector BL(pos.X - NeighborhoodRadius, pos.Y - NeighborhoodRadius, FirstPos3D.Z);
	FVector BR(pos.X + NeighborhoodRadius, pos.Y - NeighborhoodRadius, FirstPos3D.Z);
	FVector TR(pos.X + NeighborhoodRadius, pos.Y + NeighborhoodRadius, FirstPos3D.Z);
	FVector TL(pos.X - NeighborhoodRadius, pos.Y + NeighborhoodRadius, FirstPos3D.Z);
	DrawDebugLine(pWorld, BL, BR, FColor::Green, false, -1.f, 0, 2.f);
	DrawDebugLine(pWorld, BR, TR, FColor::Green, false, -1.f, 0, 2.f);
	DrawDebugLine(pWorld, TR, TL, FColor::Green, false, -1.f, 0, 2.f);
	DrawDebugLine(pWorld, TL, BL, FColor::Green, false, -1.f, 0, 2.f);
#endif
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;

	for (ASteeringAgent* pOther : Agents)
	{
		if (!pOther || pOther == pAgent) continue;

		float distance = FVector2D::Distance(pAgent->GetPosition(), pOther->GetPosition());
		if (distance <= NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = pOther;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;
	int count = GetNrOfNeighbors();
	if (count == 0) return avgPosition;

	const TArray<ASteeringAgent*>& neighbors = GetNeighbors();
	for (int i = 0; i < count; ++i)
		if (neighbors[i]) avgPosition += neighbors[i]->GetPosition();

	avgPosition /= static_cast<float>(count);
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	int count = GetNrOfNeighbors();
	if (count == 0) return avgVelocity;

	const TArray<ASteeringAgent*>& neighbors = GetNeighbors();
	for (int i = 0; i < count; ++i)
		if (neighbors[i]) avgVelocity += neighbors[i]->GetLinearVelocity();

	avgVelocity /= static_cast<float>(count);
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}
