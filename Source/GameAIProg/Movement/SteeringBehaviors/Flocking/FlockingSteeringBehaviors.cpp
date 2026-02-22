#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering;
	
	int nrOfNeighbors = pFlock->GetNrOfNeighbors();
	if (nrOfNeighbors == 0)
		return steering;
	
	FSteeringParams seekTarget{};
	seekTarget.Position = pFlock->GetAverageNeighborPos();
	Target = seekTarget;
	
	steering = Seek::CalculateSteering(deltaT, pAgent);
	
	// Debug: cyan line toward cohesion target
	if (pAgent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = pAgent.GetActorLocation();
		FVector TargetPos3D = FVector(seekTarget.Position.X, seekTarget.Position.Y, AgentPos3D.Z);
		DrawDebugLine(pAgent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Cyan, false, -1.f, 0, 2.f);
	}

	return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering;
	
	int nrOfNeighbors = pFlock->GetNrOfNeighbors();
	if (nrOfNeighbors == 0)
		return steering;

	const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();
	FVector2D separationVelocity = FVector2D::ZeroVector;

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		ASteeringAgent* pNeighbor = neighbors[i];
		if (!pNeighbor)
			continue;

		FVector2D toAgent = pAgent.GetPosition() - pNeighbor->GetPosition();
		float distance = toAgent.Length();

		if (distance > 0.f)
		{
			// y = 1/x: closer neighbors have stronger repulsion
			separationVelocity += toAgent.GetSafeNormal() * (1.f / distance);
		}
	}

	steering.LinearVelocity = separationVelocity;

	// Debug: red line showing separation direction
	if (pAgent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = pAgent.GetActorLocation();
		FVector SepVel3D = FVector(separationVelocity.X, separationVelocity.Y, 0.f);
		DrawDebugLine(pAgent.GetWorld(), AgentPos3D, AgentPos3D + SepVel3D * 50.f, FColor::Red, false, -1.f, 0, 2.f);
	}

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	int nrOfNeighbors = pFlock->GetNrOfNeighbors();
	if (nrOfNeighbors == 0)
		return steering;

	steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	// Debug: magenta line showing alignment direction
	if (pAgent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = pAgent.GetActorLocation();
		FVector VelDir3D = FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.f);
		DrawDebugLine(pAgent.GetWorld(), AgentPos3D, AgentPos3D + VelDir3D, FColor::Magenta, false, -1.f, 0, 2.f);
	}

	return steering;
}
