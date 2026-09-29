// AWAITING FABLE REVIEW - changed 2026-09-12 by Opus 5 (Drive CLAUDE.local.md section 1): kf.diff.trace logs each differential's split, so the centre's bias can be read off a run.
// AWAITING FABLE REVIEW - changed 2026-09-10 by Opus 5 (CLAUDE.local.md section 1, ADR-041): the clutch-pack LSD: bounded capacity preload + lock x |torque| replacing a per-substep velocity correction (Codex F4).
// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)


#include "VehicleDifferentialComponent.h"
#include "VehicleAxleAssemblyComponent.h"
#include "VehicleWheelComponent.h"
#include "VehicleUtilities.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarDiffTrace(TEXT("kf.diff.trace"), 0, TEXT("1 = log every differential's input and outputs at 20 Hz (Drive 2026-09-12: the Torsen split, verified)"));

// Sets default values for this component's properties
UVehicleDifferentialComponent::UVehicleDifferentialComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = false;
	// ...
}


// Called when the game starts
void UVehicleDifferentialComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void UVehicleDifferentialComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UVehicleDifferentialComponent::UpdateOutputShaft(float InDriveTorque, float InLeftAngularVelocity, float InRightAngularVelocity, float InLeftTotalInertia, float InRightTotalInertia, float InDeltaTime, float InReflectedInertia, float& OutLeftTorque, float& OutRightTorque, float& OutReflectedInertiaEachWheel)
{
	//update torque
	GetOutputTorque(
			InDriveTorque,
			InLeftAngularVelocity,
			InRightAngularVelocity,
			InLeftTotalInertia,
			InRightTotalInertia,
			InDeltaTime,
			OutLeftTorque,
			OutRightTorque
		);

	//: THE SPLIT, SAID (Drive, 2026-09-12: the owner asked that the Torsen centre "distributes power
	//: correctly automatically front rear" be VERIFIED, and no take carries a differential's outputs):
	//: `kf.diff.trace 1` logs every differential's input and two outputs at 20 Hz, with the speed
	//: difference the pack is opposing, so a split-grip run shows the bias as it happens. Off by default.
	if (CVarDiffTrace.GetValueOnAnyThread()  /* the physics thread asks */ != 0)
	{
		TraceAccumulator += InDeltaTime;
		if (TraceAccumulator >= 0.05f)
		{
			TraceAccumulator = 0.f;
			const float Total = FMath::Abs(OutLeftTorque) + FMath::Abs(OutRightTorque);
			UE_LOG(LogTemp, Log, TEXT("KF.Diff '%s': in %.0f N.m x %.2f -> left %.0f / right %.0f N.m (%.0f %% / %.0f %%), speeds %.1f / %.1f rad/s, lock %.2f/%.2f preload %.0f viscous %.1f"),
			       *GetName(), InDriveTorque, Config.GearRatio, OutLeftTorque, OutRightTorque,
			       Total > 1.f ? 100.f * FMath::Abs(OutLeftTorque) / Total : 50.f, Total > 1.f ? 100.f * FMath::Abs(OutRightTorque) / Total : 50.f,
			       InLeftAngularVelocity, InRightAngularVelocity, Config.DriveLockRatio, Config.CoastLockRatio, Config.PreloadTorque, Config.ViscousCoefficient);
		}
	}
	
	//update inertia
	OutReflectedInertiaEachWheel = 0.5 * InReflectedInertia * Config.GearRatio * Config.GearRatio;
}

void UVehicleDifferentialComponent::UpdateInputShaft(float InLeftOutputShaftAngularVelocity, float InRightOutputShaftAngularVelocity, float InLeftWheelInertia, float InRightWheelInertia, float& OutInputShaftVelocity, float& OutReflectedInertia)
{
	OutInputShaftVelocity = GetInputShaftVelocity(InLeftOutputShaftAngularVelocity, InRightOutputShaftAngularVelocity);

	OutReflectedInertia = UVehicleUtilities::SafeDivide(InLeftWheelInertia + InRightWheelInertia, Config.GearRatio * Config.GearRatio);
}

int32 UVehicleDifferentialComponent::SubstepTransferCase(
	TArrayView<UVehicleAxleAssemblyComponent* const> InAxles,
	float InSubstepDeltaTime,
	float InGearboxOutputTorque,
	float InBrakeValue,
	float InHandbrakeValue,
	bool bLineLockActive,
	float& OutTransmissionOutputShaftAngularVelocity,
	float& OutTransmissionOutputShaftEffectiveInertia,
	float& OutEffectiveDriveShaftStiffness)
{
	// 1. First iterate over all axles to gather Total Inertia, Total Angular Momentum, and Base Weights
	int32 NumOfDriveAxles = 0;
	float SumTorqueWeight = 0.f;

	float TotalDriveAxleInertia = 0.f;
	float TotalAngularMomentum = 0.f;

	for (UVehicleAxleAssemblyComponent* Axle : InAxles)
	{
		if (Axle == nullptr) continue;

		bool IsDriveAxle = Axle->GetAxleConfig().TorqueWeight > SMALL_NUMBER;
		if (IsDriveAxle)
		{
			NumOfDriveAxles++;
			SumTorqueWeight += FMath::Abs(Axle->GetAxleConfig().TorqueWeight);

			float AxleTotalInertia = Axle->GetTotalAxleInertia();
			float AxleAngVel = Axle->GetAngularVelocity();

			TotalDriveAxleInertia += AxleTotalInertia;
			TotalAngularMomentum += AxleTotalInertia * AxleAngVel;
		}
	}
	float FloatNumOfDriveAxles = (float)NumOfDriveAxles;

	// Calculate the physically correct locked angular velocity (momentum-weighted average)
	float TargetLockedAngVel = UVehicleUtilities::SafeDivide(TotalAngularMomentum, TotalDriveAxleInertia);

	// update axles
	float DriveTorque = Config.GearRatio * InGearboxOutputTorque;

	//: THE ON-DEMAND COUPLING (Drive 2026-09-25; the config's note): the drive goes to the primary axles,
	//: each secondary axle (TorqueWeight 0) takes the clutch's torque alone, the reaction off the primaries.
	//: T = clamp(ramp x (w_primary - w_secondary), +-max) as Assetto Corsa's [AWD2] computes it; the
	//: velocity-equalising amount stays the numerical cap, as for the pack below (a clutch cannot move
	//: more than would equalise the two sides this substep).
	float CouplingTorque = 0.f;
	int32 NumOfSecondaryAxles = 0;
	if (Config.bOnDemandCoupling)
	{
		float PrimaryMomentum = 0.f, PrimaryInertia = 0.f, SecondaryMomentum = 0.f, SecondaryInertia = 0.f;
		for (UVehicleAxleAssemblyComponent* Axle : InAxles)
		{
			if (Axle == nullptr) continue;
			const float AxleInertia = Axle->GetTotalAxleInertia();
			if (Axle->GetAxleConfig().TorqueWeight > SMALL_NUMBER)
			{
				PrimaryMomentum += AxleInertia * Axle->GetAngularVelocity();
				PrimaryInertia += AxleInertia;
			}
			else
			{
				SecondaryMomentum += AxleInertia * Axle->GetAngularVelocity();
				SecondaryInertia += AxleInertia;
				++NumOfSecondaryAxles;
			}
		}
		if (NumOfSecondaryAxles > 0 && PrimaryInertia > SMALL_NUMBER && SecondaryInertia > SMALL_NUMBER)
		{
			//: the secondary's speed scaled to the primary's road frame (1 = the plain propshaft comparison)
			const float SecondaryScale = FMath::Clamp(Config.CouplingSecondarySpeedScale, 0.5f, 2.f);
			const float SpeedDifference = UVehicleUtilities::SafeDivide(PrimaryMomentum, PrimaryInertia)
				- SecondaryScale * UVehicleUtilities::SafeDivide(SecondaryMomentum, SecondaryInertia);
			const float MaxTorque = FMath::Max(Config.CouplingMaxTorque, 0.f);
			CouplingTorque = FMath::Clamp(FMath::Max(Config.CouplingRampTorque, 0.f) * SpeedDifference, -MaxTorque, MaxTorque);
			//: the torque that closes the (scaled) difference this substep: d(dw)/dt = D/Ip - T/Ip - scale x T/Is. The
			//: primary's own drive D spreads the shafts during the substep, so a closed clutch must carry it too (Previs
			//: 2026-09-29: without it a pre-engaged clutch moved only the difference present at the step start, ~47 N.m on
			//: the 911's launch, and the rear ran 10 % ahead of a front that should have been clamped to it)
			const float PrimaryDrive = DriveTorque;  //: every weight is on the primaries in this mode
			const float EqualisingTorque = (UVehicleUtilities::SafeDivide(SpeedDifference, InSubstepDeltaTime)
				+ UVehicleUtilities::SafeDivide(PrimaryDrive, PrimaryInertia))
				* PrimaryInertia * SecondaryInertia / (SecondaryInertia + SecondaryScale * PrimaryInertia);
			if (FMath::Abs(CouplingTorque) > FMath::Abs(EqualisingTorque))
			{
				CouplingTorque = EqualisingTorque;
			}
			//: one-way: only while the primary runs ahead in its own direction of rotation (it drives, never brakes, the secondary)
			const float PrimarySpeed = UVehicleUtilities::SafeDivide(PrimaryMomentum, PrimaryInertia);
			if (Config.bOneWayCoupling && CouplingTorque * PrimarySpeed <= 0.f)
			{
				CouplingTorque = 0.f;
			}
		}
	}

	//: THE CENTRE IS A CLUTCH PACK TOO (Drive, 2026-09-12; ADR-041 reached the axle differentials on
	//: 2026-09-10 and never this path). What stood here moved `lock x inertia x speed difference / dt`
	//: EVERY substep - the per-substep velocity correction Codex review F4 named, whose strength rides
	//: the substep count and whose transfer knows no bound - so a 0.3 "Torsen" centre and a 0.6 Kunos
	//: pack both behaved as fully locked, and Drive's owner measured three centres identical on the
	//: pad and in a pirouette. The velocity-equalising amount stays as the NUMERICAL CAP (a pack cannot
	//: move more than would equalise the axles this substep), and the pack's own bound is put over it:
	//:
	//:     capacity = preload + lock_ratio x |input torque|
	//:
	//: scaled uniformly across the axles so their biases still sum to zero. A Torsen with a 4:1 bias
	//: ratio is lock 0.30 and no preload: the gripping axle takes up to 80 % of the drive and no more.
	//: THE VISCOUS COUPLING (Drive 2026-09-24): c x |speed difference across the coupling| joins the
	//: capacity. Two drive axles' speed difference is the sum of their distances to the locked speed
	//: (they sit on opposite sides of it); more axles sum the same way. A coupling with no plates has
	//: lock 0, whose raw here would be zero, so a stated coefficient makes the raw the EQUALISING
	//: torque - the axle law's - and the capacity bounds it. No coefficient: this block as it was.
	const bool bViscous = Config.ViscousCoefficient > 0.f;
	float RawBiasMax = 0.f;
	float SpeedSpread = 0.f;
	{
		const bool bIsDrivePass = (DriveTorque * TargetLockedAngVel) >= 0.f;
		const float PassLockRatio = bViscous ? 1.f : (bIsDrivePass ? Config.DriveLockRatio : Config.CoastLockRatio);
		for (UVehicleAxleAssemblyComponent* Axle : InAxles)
		{
			if (Axle == nullptr || !(Axle->GetAxleConfig().TorqueWeight > SMALL_NUMBER)) continue;
			const float Difference = TargetLockedAngVel - Axle->GetAngularVelocity();
			SpeedSpread += FMath::Abs(Difference);
			const float Raw = UVehicleUtilities::SafeDivide(Difference * Axle->GetTotalAxleInertia() * PassLockRatio, InSubstepDeltaTime);
			RawBiasMax = FMath::Max(RawBiasMax, FMath::Abs(Raw));
		}
	}
	const float ViscousCapacity = bViscous ? Config.ViscousCoefficient * SpeedSpread : 0.f;
	const float PackCapacity = FMath::Max(Config.PreloadTorque, 0.f) + FMath::Clamp(Config.DriveLockRatio, 0.f, 1.f) * FMath::Abs(DriveTorque) + ViscousCapacity;
	const float PackCapacityCoast = FMath::Max(Config.PreloadTorque, 0.f) + FMath::Clamp(Config.CoastLockRatio, 0.f, 1.f) * FMath::Abs(DriveTorque) + ViscousCapacity;
	
	float SumAngVel = 0.f;
	float SumDriveAxleInertia = 0.f;
	float SumStiffness = 0.f;

	for (UVehicleAxleAssemblyComponent* Axle : InAxles)
	{
		if (Axle == nullptr) continue;

		float AxleInertia = 0.f;
		float AxleAngVel = 0.f;
		float AxleStiffness = 0.f;

		bool IsDriveAxle = Axle->GetAxleConfig().TorqueWeight > SMALL_NUMBER;
		if (IsDriveAxle)
		{
			// central diff locking logic
			// Use TargetLockedAngVel instead of simple arithmetic average
			float AngVelDifference = TargetLockedAngVel - Axle->GetAngularVelocity();

			bool bIsDrive = (DriveTorque * TargetLockedAngVel) >= 0.f;
			float CurrentLockRatio = bViscous ? 1.f : (bIsDrive ? Config.DriveLockRatio : Config.CoastLockRatio);

			// The calculated TorqueBias is now guaranteed to sum to exactly 0 across all axles
			float TorqueBias = UVehicleUtilities::SafeDivide(AngVelDifference * Axle->GetTotalAxleInertia() * CurrentLockRatio, InSubstepDeltaTime);
			//: the pack's bound (Drive 2026-09-12): the same scale on every axle keeps the sum at zero
			{
				const float Capacity = bIsDrive ? PackCapacity : PackCapacityCoast;
				if (RawBiasMax > Capacity && RawBiasMax > SMALL_NUMBER)
				{
					TorqueBias *= Capacity / RawBiasMax;
				}
			}
			float NormTorqueWeight = UVehicleUtilities::SafeDivide(Axle->GetAxleConfig().TorqueWeight, SumTorqueWeight);

			// Combine mechanical static split + LSD clutch pack transfer (- the on-demand coupling's reaction)
			float AxleDriveTorque = DriveTorque * NormTorqueWeight + TorqueBias - CouplingTorque * NormTorqueWeight;
			if (CVarDiffTrace.GetValueOnAnyThread()  /* the physics thread asks */ != 0)
			{
				TraceAccumulator += InSubstepDeltaTime;
				if (TraceAccumulator >= 0.05f)
				{
					TraceAccumulator = 0.f;
					UE_LOG(LogTemp, Log, TEXT("KF.Centre '%s': in %.0f N.m -> axle '%s' %.0f N.m (%.0f %% of the input; static share %.0f %%, pack bias %+.0f of capacity %.0f), axle %.1f rad/s vs locked %.1f, lock %.2f/%.2f preload %.0f viscous %.1f"),
					       *GetName(), DriveTorque, *Axle->GetName(), AxleDriveTorque,
					       FMath::Abs(DriveTorque) > 1.f ? 100.f * AxleDriveTorque / DriveTorque : 0.f, 100.f * NormTorqueWeight, TorqueBias,
					       bIsDrive ? PackCapacity : PackCapacityCoast, Axle->GetAngularVelocity(), TargetLockedAngVel,
					       Config.DriveLockRatio, Config.CoastLockRatio, Config.PreloadTorque, Config.ViscousCoefficient);
				}
			}

			// burnout assist
			bool IsMainDriveAxle = NormTorqueWeight > 0.5f;
			bool ShouldReleaseBrake = IsMainDriveAxle && bLineLockActive;

			Axle->SubstepAxle(
				InSubstepDeltaTime,
				AxleDriveTorque,
				InBrakeValue * !ShouldReleaseBrake,
				InHandbrakeValue,
				AxleInertia, AxleAngVel, AxleStiffness
			);

			SumAngVel += AxleAngVel;
			SumDriveAxleInertia += AxleInertia;
			SumStiffness += AxleStiffness;
		}
		else
		{
			//: a secondary axle of the on-demand coupling takes the clutch's torque (zero otherwise) - and is POWERED for it
			Axle->SetCouplingFed(Config.bOnDemandCoupling);
			Axle->SubstepAxle(
				InSubstepDeltaTime,
				NumOfSecondaryAxles > 0 ? CouplingTorque / NumOfSecondaryAxles : 0.f,
				InBrakeValue,
				InHandbrakeValue,
				AxleInertia, AxleAngVel, AxleStiffness
			);
		}
	}

	// For the output shaft, arithmetic mean is generally fine for RPM display/feedback, 
	// though TargetLockedAngVel * Config.GearRatio is also physically valid if fully locked.
	OutTransmissionOutputShaftAngularVelocity = UVehicleUtilities::SafeDivide(SumAngVel * Config.GearRatio, FloatNumOfDriveAxles);
	
	const float GearRatioSquareInv = UVehicleUtilities::SafeDivide(1.f, Config.GearRatio * Config.GearRatio);
	OutTransmissionOutputShaftEffectiveInertia = SumDriveAxleInertia * GearRatioSquareInv;
	OutEffectiveDriveShaftStiffness = SumStiffness * GearRatioSquareInv;

	return NumOfDriveAxles;
}

int32 UVehicleDifferentialComponent::UpdateTransferCase(
	const TArray<UVehicleAxleAssemblyComponent*>& InAxles, 
	float InDeltaTime,
	float InGearboxOutputTorque, 
	float InReflectedInertia,
	float InBrakeValue, 
	float InHandbrakeValue, 
	float InSteeringValue, 
	bool bLineLockActive,
	float& OutTransmissionOutputShaftAngularVelocity,
	float& OutTransmissionOutputShaftEffectiveInertia,
	float& OutEffectiveDriveShaftStiffness)
{
	for (UVehicleAxleAssemblyComponent* Axle : InAxles)
	{
		if (IsValid(Axle))
		{
			Axle->PreStepAxle(InDeltaTime, InSteeringValue);
		}
	}

	int32 NumOfDriveAxles = SubstepTransferCase(
		InAxles, InDeltaTime, InGearboxOutputTorque,
		InBrakeValue, InHandbrakeValue, bLineLockActive,
		OutTransmissionOutputShaftAngularVelocity,
		OutTransmissionOutputShaftEffectiveInertia,
		OutEffectiveDriveShaftStiffness
	);

	for (UVehicleAxleAssemblyComponent* Axle : InAxles)
	{
		if (IsValid(Axle))
		{
			Axle->PostStepAxle();
		}
	}

	//return the number of drive axles
	return NumOfDriveAxles;
}

float UVehicleDifferentialComponent::CalculateEffectiveWheelRadius(const TArray<UVehicleAxleAssemblyComponent*>& InAxles)
{
	return CalculateEffectiveWheelRadius_Internal(InAxles);
}

float UVehicleDifferentialComponent::CalculateEffectiveWheelRadius_Internal(
	TArrayView<UVehicleAxleAssemblyComponent* const> InAxles)
{
	float effectiveR = 0.f;
	int32 driveAxleNum = 0;
	for (UVehicleAxleAssemblyComponent* Axle : InAxles)
	{
		if (!IsValid(Axle))continue;

		if (Axle->GetAxleConfig().TorqueWeight > 0)
		{
			UVehicleWheelComponent* LeftWheel;
			UVehicleWheelComponent* RightWheel;
			UVehicleDifferentialComponent* Diff;

			Axle->GetWheels(LeftWheel, RightWheel);
			Axle->GetDifferential(Diff);

			float SumR = 0.f;
			int32 n = 0;
			if (IsValid(LeftWheel))
			{
				SumR += LeftWheel->GetWheelConfig().Radius;
				n++;
			}
			if (IsValid(RightWheel))
			{
				SumR += RightWheel->GetWheelConfig().Radius;
				n++;
			}
			float avgR = UVehicleUtilities::SafeDivide(SumR, (float)n);
			effectiveR += UVehicleUtilities::SafeDivide(avgR, Diff->Config.GearRatio);

			driveAxleNum++;
		}
	}
	effectiveR = UVehicleUtilities::SafeDivide(effectiveR, (float)driveAxleNum * Config.GearRatio);

	return effectiveR;
}

void UVehicleDifferentialComponent::GetOutputTorque(
	float InTorque,
	float InLeftAngularVelocity,
	float InRightAngularVelocity, 
	float InLeftTotalInertia,
	float InRightTotalInertia,
	float InDeltaTime,
	float& OutLeftTorque,
	float& OutRightTorque)
{
	float OpenDiffTorque = 0.5 * Config.GearRatio * InTorque;

	float AverageAngularVelocity = 0.5 * (InLeftAngularVelocity + InRightAngularVelocity);
	bool bIsDrive = (OpenDiffTorque * AverageAngularVelocity) >= 0.f;
	float CurrentLockRatio = bIsDrive ? Config.DriveLockRatio : Config.CoastLockRatio;

	float ReducedInertia = UVehicleUtilities::SafeDivide(
		InLeftTotalInertia * InRightTotalInertia, InLeftTotalInertia + InRightTotalInertia);

	float OmegaDiff = InLeftAngularVelocity - InRightAngularVelocity;

	// A CLUTCH-PACK LIMITED SLIP, not a per-substep velocity correction (2026-09-10, Codex review
	// F4). What stood here removed CurrentLockRatio of the speed difference EVERY SUBSTEP:
	//
	//     TransferTorque = ReducedInertia * OmegaDiff / dt * CurrentLockRatio
	//
	// so the difference decayed as (1 - lock)^n and the strength depended on the SUBSTEP COUNT
	// rather than on the differential. At 240 Hz a stated 0.6 leaves 0.4 % of the difference after
	// 25 ms: every partially-locked differential in the fleet behaved as fully locked. Drive's owner
	// found it from the driver's seat on an Audi Sport Quattro S1 E2 - three differentials stating
	// 0.4 to 0.6, a car that would not rotate - before reading any of this code.
	//
	// The pack instead carries a BOUNDED torque, which is what a clutch pack physically does:
	//
	//     capacity = preload + lock_ratio * |input torque|
	//
	// preload holds it together with no input at all, and the ratio is the torque-sensitive part.
	// The transfer opposes the speed difference, so its work is never positive and the two outputs
	// stay equal and opposite. None of that mentions dt.
	//
	// The old expression survives as a NUMERICAL CAP and nothing more: a pack cannot transfer more
	// than would equalise the two shafts within this step, or it overshoots and rings.
	//
	// A VISCOUS COUPLING (Drive 2026-09-24) adds c x |speed difference| to the same capacity: a
	// coupling with no plates states a coefficient alone and transfers exactly that (the equalising
	// cap is far larger at any realistic c); a plate-and-viscous unit states both and gets the sum.
	const float Capacity = FMath::Max(Config.PreloadTorque, 0.f)
		+ FMath::Clamp(CurrentLockRatio, 0.f, 1.f) * FMath::Abs(Config.GearRatio * InTorque)
		+ FMath::Max(Config.ViscousCoefficient, 0.f) * FMath::Abs(OmegaDiff);
	const float NoOvershoot = FMath::Abs(
		UVehicleUtilities::SafeDivide(ReducedInertia * OmegaDiff, InDeltaTime));
	const float TransferTorque = FMath::Sign(OmegaDiff) * FMath::Min(Capacity, NoOvershoot);

	OutLeftTorque = OpenDiffTorque - TransferTorque;
	OutRightTorque = OpenDiffTorque + TransferTorque;

}

void UVehicleDifferentialComponent::GetOpenDiffOutputTorque(float InTorque, float& OutTorqueLeft, float& OutTorqueRight)
{
	float OpenDiffTorque = 0.5 * Config.GearRatio * InTorque;
	OutTorqueLeft = OpenDiffTorque;
	OutTorqueRight = OpenDiffTorque;
}

float UVehicleDifferentialComponent::GetInputShaftVelocity(float OutputShaftAngularVelocityLeft, float OutputShaftAngularVelocityRight)
{
	return (OutputShaftAngularVelocityLeft + OutputShaftAngularVelocityRight) * 0.5 * Config.GearRatio;
}