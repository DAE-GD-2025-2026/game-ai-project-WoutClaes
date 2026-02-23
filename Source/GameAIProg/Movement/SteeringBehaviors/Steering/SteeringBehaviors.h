#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

inline float ToRadians(const float degrees)
{
	return degrees * PI / 180.f;
}

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	virtual bool UsesManualRotation() const { return false; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }
	
	virtual const char* GetName() const = 0;

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
//----------
// Seek
//----------
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Seek"; }
};

//----------
// Flee
//----------
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;

	
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Flee"; }
};

//----------
// Arrive
//----------
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Arrive"; }
	
private:
	float slowRadius {1000.f};
	float targetRadius {100.f};
};

//----------
// Face
//----------
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Face"; }
	
	virtual bool UsesManualRotation() const override { return true; }
};

//----------
// Pursuit
//----------
class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Pursuit"; }
};

//----------
// Evade
//----------
class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Evade"; }
};

//----------
// Wander
//----------
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;
	
	// steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	// debug rendering
	virtual const char* GetName() const override { return "Wander"; }
	
	void SetWanderOffset(const float offset) {m_OffsetDistance = offset;};
	void SetWanderRadius(const float radius) {m_Radius = radius;};
	void SetMaxAngleChange(const float rad) {m_MaxAngleChange = rad;};
	
protected:
	float m_OffsetDistance{6.f};
	float m_Radius{4.f};
	float m_MaxAngleChange {ToRadians(45)};
	float m_WanderAngle {0.f};
};