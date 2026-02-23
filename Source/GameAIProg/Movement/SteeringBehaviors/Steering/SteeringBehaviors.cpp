#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);

		// Yellow line to target
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Yellow, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
	}

	return steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);

		// Red line to target (danger)
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Red, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
	}
	
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
		speed = distance / slowRadius;
	
	steering.LinearVelocity = direction * speed;

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);

		// Yellow line to target
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Yellow, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
		// Orange circle: slow radius
		DrawDebugCircle(Agent.GetWorld(), Agent.GetActorLocation(), slowRadius, 32, FColor::Orange, false, -1.f, 0, 2.f, FVector(0,1,0), FVector(1,0,0));
		// White circle: target radius
		DrawDebugCircle(Agent.GetWorld(), Agent.GetActorLocation(), targetRadius, 32, FColor::White, false, -1.f, 0, 2.f, FVector(0,1,0), FVector(1,0,0));
	}
	
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

	float desiredAngle = FMath::RadiansToDegrees(atan2(toTarget.Y, toTarget.X));
	float currentAngle = Agent.GetRotation();
	float angleDifference = desiredAngle - currentAngle;

	steering.AngularVelocity = angleDifference / 180.f;

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);

		// Yellow line to target
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Yellow, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
	}

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

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);
		FVector PredictedPos3D = FVector(predictedPos.X, predictedPos.Y, AgentPos3D.Z);

		// Yellow line to current target position
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Yellow, false, -1.f, 0, 2.f);
		// Cyan line to predicted position
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, PredictedPos3D, FColor::Cyan, false, -1.f, 0, 2.f);
		// Cyan sphere at predicted position
		DrawDebugSphere(Agent.GetWorld(), PredictedPos3D, 20.f, 8, FColor::Cyan, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
	}
	
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

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector TargetPos3D = FVector(Target.Position.X, Target.Position.Y, AgentPos3D.Z);
		FVector PredictedPos3D = FVector(predictedPos.X, predictedPos.Y, AgentPos3D.Z);

		// Red line to current target position (danger)
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, TargetPos3D, FColor::Red, false, -1.f, 0, 2.f);
		// Orange line to predicted position
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, PredictedPos3D, FColor::Orange, false, -1.f, 0, 2.f);
		// Orange sphere at predicted position
		DrawDebugSphere(Agent.GetWorld(), PredictedPos3D, 20.f, 8, FColor::Orange, false, -1.f, 0, 2.f);
		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
	}
	
	return steering;
}

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	float t = (float)rand() / (float)RAND_MAX;
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

	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector CircleCenter3D = FVector(circleCenter.X, circleCenter.Y, AgentPos3D.Z);
		FVector WanderTarget3D = FVector(wanderTarget.X, wanderTarget.Y, AgentPos3D.Z);

		// Green line: current facing direction
		DrawDebugLine(Agent.GetWorld(), AgentPos3D, AgentPos3D + Agent.GetActorForwardVector() * 150.f, FColor::Green, false, -1.f, 0, 3.f);
		// White circle: wander circle
		DrawDebugCircle(Agent.GetWorld(), CircleCenter3D, m_Radius, 32, FColor::White, false, -1.f, 0, 2.f, FVector(0,1,0), FVector(1,0,0));
		// Yellow sphere: current wander target on circle
		DrawDebugSphere(Agent.GetWorld(), WanderTarget3D, 10.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
	}

	return steering;
}