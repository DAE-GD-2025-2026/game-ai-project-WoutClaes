#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(2000.f);
	TrimWorld->bShouldTrimWorld = true;

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

	pPrey = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector(0.f, 0.f, 90.f), FRotator::ZeroRotator, SpawnParams);
	if (pPrey)
	{
		pPreyWander = std::make_unique<Wander>();
		pPrey->SetSteeringBehavior(pPreyWander.get());
	}

	for (int i = 0; i < NrOfWolves; ++i)
	{
		FVector SpawnLoc(FMath::RandRange(-500.f, 500.f), FMath::RandRange(-500.f, 500.f), 90.f);
		ASteeringAgent* NewWolf = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, SpawnLoc, FRotator::ZeroRotator, SpawnParams);
		
		if (NewWolf)
		{
			Wolves.Add(NewWolf);

			auto pursuit = std::make_unique<Pursuit>();
			auto flee = std::make_unique<Flee>();

			std::vector<BlendedSteering::WeightedBehavior> behaviors = {
				{ pursuit.get(), 0.8f },
				{ flee.get(),    0.0f }
			};

			auto blend = std::make_unique<BlendedSteering>(behaviors);
			NewWolf->SetSteeringBehavior(blend.get());

			WolfPursuits.Add(std::move(pursuit));
			WolfFlees.Add(std::move(flee));
			WolfBlends.Add(std::move(blend));
		}
	}
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		ImGui::End();
		
#pragma endregion
	
		// --- WOLF PACK LOGIC ---
	if (!pPrey) return;

	float EncircleRadius = 250.f;
	float OrbitSpeed = 1.2f;

	FVector PreyForward3D = pPrey->GetActorForwardVector();
	FVector2D PreyForward(PreyForward3D.X, PreyForward3D.Y);
	if (PreyForward.IsNearlyZero()) PreyForward = FVector2D(1.f, 0.f);
	PreyForward.Normalize();
	
	FVector2D PreyRight(-PreyForward.Y, PreyForward.X);

	for (int i = 0; i < Wolves.Num(); ++i)
	{
		if (!Wolves[i]) continue;

		Wolves[i]->SetDebugRenderingEnabled(CanDebugRender);

		float BaseAngle = (2.f * PI / Wolves.Num()) * i;
		float TimeAngle = GetWorld()->GetTimeSeconds() * OrbitSpeed;
		float TotalAngle = BaseAngle + TimeAngle;

		FVector2D LocalOffset(cos(TotalAngle) * EncircleRadius, sin(TotalAngle) * EncircleRadius);
		
		FVector2D WorldSlotOffset = (PreyForward * LocalOffset.X) + (PreyRight * LocalOffset.Y);

		FTargetData PreyTarget;
		PreyTarget.Position = pPrey->GetPosition() + WorldSlotOffset;
		PreyTarget.LinearVelocity = pPrey->GetLinearVelocity();
		
		WolfPursuits[i]->SetTarget(PreyTarget);

		float ClosestDist = 999999.f;
		ASteeringAgent* ClosestWolf = nullptr;

		for (int j = 0; j < Wolves.Num(); ++j)
		{
			if (i == j || !Wolves[j]) continue;

			float Dist = FVector2D::Distance(Wolves[i]->GetPosition(), Wolves[j]->GetPosition());
			if (Dist < ClosestDist)
			{
				ClosestDist = Dist;
				ClosestWolf = Wolves[j];
			}
		}

		float* pFleeWeight = WolfBlends[i]->GetWeight(WolfFlees[i].get());
		float* pPursuitWeight = WolfBlends[i]->GetWeight(WolfPursuits[i].get());

		if (ClosestWolf && ClosestDist < 180.f) 
		{
			FTargetData FleeTarget;
			FleeTarget.Position = ClosestWolf->GetPosition();
			WolfFlees[i]->SetTarget(FleeTarget);

			if (pFleeWeight) *pFleeWeight = 0.4f; 
			if (pPursuitWeight) *pPursuitWeight = 0.6f; 
		}
		else
		{
			if (pFleeWeight) *pFleeWeight = 0.0f;
			if (pPursuitWeight) *pPursuitWeight = 0.8f;
		}
	}

	pPrey->SetDebugRenderingEnabled(CanDebugRender);
	}
}
