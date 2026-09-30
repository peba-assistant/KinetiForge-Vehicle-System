// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "VehicleWheelStructs.h"
#include "VehicleSuspensionStructs.h"

class UVehicleWheelComponent;
class UCurveFloat;

/**
 * 
 */
class KINETIFORGE_API FVehicleWheelSolver
{
public:
	FVehicleWheelSolver();
	~FVehicleWheelSolver();

	void Initialize(const FVehicleTireConfig& TireConfig);

	void PreStep(
		float InMacroDeltaTime,
		const FTransform& AsyncChassisWorldTransform,
		const FVehicleSuspensionSimState& SuspensionState,
		const FVehicleWheelConfig& WheelConfig,
		const FVehicleTireConfig& TireConfig
	);

	void Substep(
		float InSubstepDeltaTime,
		float InDriveTorque,
		float InBrakeTorque,
		float InHandbrakeTorque,
		float InReflectedInertia,
		const FVehicleWheelConfig& WheelConfig,
		const FVehicleTireConfig& TireConfig,
		const FVehicleABSConfig& ABSConfig,
		const FVehicleSuspensionSimState& SuspensionState
	);

	void PostStep(

	);

	void DrawWheelForce(
		UVehicleWheelComponent* WheelComponent,
		const FVehicleSuspensionSimState& SuspensionState,
		float Duration = -1,
		float Thickness = 5,
		float Length = 1,
		bool bDrawVelocity = true,
		bool bDrawSlip = true,
		bool bDrawInertia = true);
	void UpdateCachedLUTs(const FVehicleTireConfig& Config);

	static float GetTangentAtOrigin(const FRichCurve& Curve);

	// all data during simulation
	FVehicleWheelSimState State;
	FVehicleWheelSimContext CurrentContext;
	FVehicleWheelCachedLUTs CachedLUTs;

	// Previs 2026-09-30 (AWAITING FABLE REVIEW): one PI step of the ABS on the slip error e = |slip| - target; the integrator
	// is clamped to [0, 1] (anti-windup) and the returned brake fraction u is the integrator less Kp e, clamped to [0, 1].
	static float AbsPiStep(float& Integrator, const float SlipError, const float Kp, const float Ki, const float Dt)
	{
		Integrator = FMath::Clamp(Integrator - Ki * SlipError * Dt, 0.f, 1.f);
		return FMath::Clamp(Integrator - Kp * SlipError, 0.f, 1.f);
	}

protected:

private:
	static void PredictSlipAndUpdateABS(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const FVehicleABSConfig& ABSConfig,
		const float TargetBrakeTorque,
		const bool bOnGround);
	static void UpdateDynamicFrictionMultiplier(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const FVehicleTireConfig& TireConfig,
		const float ImpactFriction);
	static void UpdateLinearVelocity(
		FVehicleWheelSimState& LocalState,
		const FVector3f& LongForceDir,
		const FVector3f& LatForceDir,
		const FVector3f& ImpactPointWorldVelocity);
	static void UpdateSlipVelocity(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const bool bOnGround
	);
	static void UpdateSlipAngle(
		FVehicleWheelSimState& LocalState,
		const bool bOnGround);
	static void UpdateSlipRatio(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const bool bOnGround);
	/**Returns transient slip ratio and slip angle (normalized)*/
	static FVector2f UpdateTransientSlip(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const bool bOnGround,
		const FVector2f& RelaxationLength);
	static FVector2f CalculateGravityCompensationOnSlope(
		FVehicleWheelSimState& LocalState,
		FVehicleWheelSimContext& Context,
		const float PositiveForceIntoSurface,
		const bool bOnGround,
		const FVector3f& LongForceDir,
		const FVector3f& LatForceDir);
	static float CalculateAvailableGrip(
		const float FrictionMultiplier,
		const float StaticSprungMass,
		const float WheelLoad,
		const float Saturation,
		const float ReferenceLoad = 0.f,
		const float DoubleLoadForceRatio = 0.f);
	static FVector2f SolveTireForce(
		FVehicleWheelSimState& LocalState,
		const FVehicleWheelSimContext& Context,
		const float StaticSprungMass,
		const float EffectiveSprungMassLong,
		const float EffectiveSprungMassLat,
		const float PositiveForceIntoSurface,
		const bool bOnGround,
		const FVehicleTireConfig& TireConfig,
		const FVehicleWheelCachedLUTs& TireLUTs);
};
