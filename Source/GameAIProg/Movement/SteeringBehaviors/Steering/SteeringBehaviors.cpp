#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	// Add debug rendering

	return steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	
	// Add debug rendering
	
	return steering;
}

//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Length();
	
	if (distance < targetRadius)
	{
		steering.LinearVelocity = FVector2D::ZeroVector;
		return steering;
	}
	
	FVector2D direction = toTarget.GetSafeNormal();
	
	float speed = 1.f;
	
	if (distance < slowRadius)
	{
		speed = distance / slowRadius;
	}
	
	steering.LinearVelocity = direction * speed;
	return steering;
}

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	
	if (toTarget.IsNearlyZero())
	{
		steering.AngularVelocity = 0.f;
		return steering;
	}
	
	float desiredAngle = atan2(toTarget.Y, toTarget.X);
	float currentAngle = Agent.GetRotation();
	float angleDifference = desiredAngle - currentAngle;
	
	steering.AngularVelocity = angleDifference;
	
	return steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Length();
	
	if (toTarget.IsNearlyZero())
	{
		steering.LinearVelocity = toTarget;
		return steering;
	}
	
	float speed = Agent.GetMaxLinearSpeed();
	float predictionTime = distance / speed;

	FVector2D predictedPos = Target.Position + Target.LinearVelocity * predictionTime;

	steering.LinearVelocity = predictedPos - Agent.GetPosition();
	
	return steering;
}

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Length();
	
	if (toTarget.IsNearlyZero())
	{
		steering.LinearVelocity = Agent.GetPosition() - Target.Position;
		return steering;
	}
	
	float speed = Agent.GetMaxLinearSpeed();
	float predictionTime = distance / speed;

	FVector2D predictedPos = Target.Position + Target.LinearVelocity * predictionTime;

	steering.LinearVelocity = Agent.GetPosition() - predictedPos;
	
	return steering;
}

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	return steering;
}