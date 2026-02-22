#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


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
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLoc, FRotator::ZeroRotator);
	}

	Neighbors.SetNum(FlockSize);

	pCohesionBehavior    = std::make_unique<Cohesion>(this);
	pSeparationBehavior  = std::make_unique<Separation>(this);
	pVelMatchBehavior    = std::make_unique<VelocityMatch>(this);
	pSeekBehavior        = std::make_unique<Seek>();
	pWanderBehavior      = std::make_unique<Wander>();
	pEvadeBehavior       = std::make_unique<Evade>();

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
	{
		if (pAgent)
			pAgent->SetSteeringBehavior(pPrioritySteering.get());
	}
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
	if (pAgentToEvade)
	{
		FSteeringParams evadeTarget{};
		evadeTarget.Position       = pAgentToEvade->GetPosition();
		evadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(evadeTarget);
	}

	for (ASteeringAgent* pAgent : Agents)
	{
		if (!pAgent) continue;

		RegisterNeighbors(pAgent);

		pAgent->Tick(DeltaTime);
	}

	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::RenderDebug()
{
	if (!DebugRenderSteering)
		return;

	for (ASteeringAgent* pAgent : Agents)
	{
		if (pAgent)
			pAgent->SetDebugRenderingEnabled(true);
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
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
		
		// --- Debug rendering toggles ---
		if (ImGui::Checkbox("Debug Steering", &DebugRenderSteering))
		{
			bool bEnable = DebugRenderSteering;
			for (ASteeringAgent* pAgent : Agents)
				if (pAgent) pAgent->SetDebugRenderingEnabled(bEnable);
		}
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Partitions",   &DebugRenderPartitions);

		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		// --- Weight sliders (copy/compare pattern for Unreal safety) ---
		auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		//Cohesion
		float w = Behaviors[0].Weight;
		if (ImGui::SliderFloat("Cohesion",    &w, 0.f, 1.f, "%.2f")) Behaviors[0].Weight = w;
		//Separation
		w = Behaviors[1].Weight;
		if (ImGui::SliderFloat("Separation",  &w, 0.f, 1.f, "%.2f")) Behaviors[1].Weight = w;
		//Velocity Match
		w = Behaviors[2].Weight;
		if (ImGui::SliderFloat("Alignment",   &w, 0.f, 1.f, "%.2f")) Behaviors[2].Weight = w;
		//Seek
		w = Behaviors[3].Weight;
		if (ImGui::SliderFloat("Seek",        &w, 0.f, 1.f, "%.2f")) Behaviors[3].Weight = w;
		//Wander
		w = Behaviors[4].Weight;
		if (ImGui::SliderFloat("Wander",      &w, 0.f, 1.f, "%.2f")) Behaviors[4].Weight = w;

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

	// Draw the neighborhood radius circle (white)
	DrawDebugCircle(pWorld, FirstPos3D, NeighborhoodRadius, 32,
		FColor::White, false, -1.f, 0, 3.f,
		FVector(0,1,0), FVector(1,0,0));

	// Highlight each neighbor with a small sphere (yellow)
	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		ASteeringAgent* pNeighbor = Neighbors[i];
		if (pNeighbor)
		{
			DrawDebugSphere(pWorld, pNeighbor->GetActorLocation(),
				30.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
		}
	}
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
	if (NrOfNeighbors == 0) return avgPosition;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (Neighbors[i])
			avgPosition += Neighbors[i]->GetPosition();
	}
	avgPosition /= static_cast<float>(NrOfNeighbors);
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return avgVelocity;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (Neighbors[i])
			avgVelocity += Neighbors[i]->GetLinearVelocity();
	}
	avgVelocity /= static_cast<float>(NrOfNeighbors);
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}
