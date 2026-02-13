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
	
	float t = (float)rand() / (float)RAND_MAX; // 0–1
	float randomChange = (t * 2.0f - 1.0f) * m_MaxAngleChange;
	m_WanderAngle += randomChange;

	FVector forward3D = Agent.GetActorForwardVector();
	FVector2D forward(forward3D.X, forward3D.Y);

	FVector2D circleCenter = Agent.GetPosition() + forward * m_OffsetDistance;
	
	FVector2D offset;
	offset.X = cos(m_WanderAngle) * m_Radius;
	offset.Y = sin(m_WanderAngle) * m_Radius;

	FVector2D wanderTarget = circleCenter + offset;

	steering.LinearVelocity = wanderTarget - Agent.GetPosition();
	steering.AngularVelocity = 0.f;

	return steering;
}