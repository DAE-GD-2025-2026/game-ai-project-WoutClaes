
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	float TotalWeight = 0.f;
	
	for (WeightedBehavior& WeightedBeh : WeightedBehaviors)
	{
		if (WeightedBeh.pBehavior && WeightedBeh.Weight > 0.f)
		{
			SteeringOutput BehaviorOutput = WeightedBeh.pBehavior->CalculateSteering(DeltaT, Agent);
			BlendedSteering.LinearVelocity  += BehaviorOutput.LinearVelocity  * WeightedBeh.Weight;
			BlendedSteering.AngularVelocity += BehaviorOutput.AngularVelocity * WeightedBeh.Weight;
			TotalWeight += WeightedBeh.Weight;
		}
	}
	
	if (TotalWeight > 0.f)
	{
		BlendedSteering.LinearVelocity  /= TotalWeight;
		BlendedSteering.AngularVelocity /= TotalWeight;
	}
	
	// Debug: draw the blended velocity as a white line
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector BlendedVel3D = FVector(BlendedSteering.LinearVelocity.X, BlendedSteering.LinearVelocity.Y, 0.f);
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + BlendedVel3D, FColor::White, false, -1.f, 0, 3.f);
	}
	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}