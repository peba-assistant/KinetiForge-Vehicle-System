// AWAITING FABLE REVIEW - changed 2026-09-10 by Opus 5 (CLAUDE.local.md section 1, ADR-041): explicit gear ratios survive the dirty check (Codex F1).
// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)


#include "VehicleGearboxComponent.h"
#include "VehicleUtilities.h"

// Sets default values for this component's properties
UVehicleGearboxComponent::UVehicleGearboxComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = false;
	// ...
}


// Called when the game starts
void UVehicleGearboxComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	//: F1: BeginPlay also calls the synthesiser, so it must not run once explicit ratios are in.
	if (!bHasExplicitGearRatios) CalculateGearRatios();	//to avoid nullptr
}

void UVehicleGearboxComponent::StartShift(int32 InTargetGear, bool bImmediate)
{
	//check if gearratios has to be calculated again
	if (IsGearDataDirty())
	{
		CalculateGearRatios();
	}

	//check if target gear is valid
	if (InTargetGear == CurrentGear
		|| InTargetGear > GearRatios.Num()
		|| InTargetGear < -ReverseGearRatios.Num()
		|| !bIsInGear)
		return;

	//if GetWorld()=false, can't set timer, if ShiftDelay to small, immediate
	bImmediate = bImmediate || !IsValid(GetWorld()) || Config.ShiftDelay < SMALL_NUMBER;

	//Sequential gearbox can only shift one gear once
	if (Config.bSequentialGearbox)InTargetGear = FMath::Clamp(InTargetGear, CurrentGear - 1, CurrentGear + 1);

	//prepare shift
	TargetGear = InTargetGear;
	CurrentGearRatio = 0.f;
	bIsInGear = false;
	int32 AbsTargetGear = FMath::Abs(TargetGear);
	int32 AbsCurrentGear = FMath::Abs(CurrentGear);
	bool bShiftingUp = AbsTargetGear > AbsCurrentGear;
	bool bShiftingDown = AbsTargetGear < AbsCurrentGear;
	bShouldRevMatch = bShiftingDown && TargetGear != 0;
	bShouldCutSpark = Config.bSequentialGearbox && bShiftingUp && AbsTargetGear != 1;

	//start shift
	if (bImmediate)
	{
		FinalizeShift();
	}
	else
	{
		//set timer
		GetWorld()->GetTimerManager().SetTimer(
			GearShiftTimerHandle,
			this,
			&UVehicleGearboxComponent::FinalizeShift,	//call this function after delay
			Config.ShiftDelay,
			false // non cycle
		);
	}
}

void UVehicleGearboxComponent::FinalizeShift()
{
	CurrentGear = TargetGear;
	bIsInGear = true;
	bShouldRevMatch = false;
	bShouldCutSpark = false;
	CurrentGearRatio = GetGearRatio(CurrentGear);
	TargetGear = 0;

	if (ShiftFinishedCallback.IsBound())
	{
		ShiftFinishedCallback.Execute();
		ShiftFinishedCallback.Unbind();	//if it is not manually unbound, the delegate will be called after every shift
	}
}

// Called every frame
void UVehicleGearboxComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UVehicleGearboxComponent::ShiftToTargetGear(int32 InTargetGear, bool bImmediate)
{
	StartShift(InTargetGear, bImmediate);
}

void UVehicleGearboxComponent::ShiftToTargetGearWithDelegate(FOnShiftFinishedDelegate InOnShiftFinished, int32 InTargetGear, bool bImmediate)
{
	ShiftFinishedCallback = InOnShiftFinished;
	StartShift(InTargetGear, bImmediate);
}

void UVehicleGearboxComponent::UpdateOutputShaft(
	float InClutchTorque, 
	float& OutTorque
)
{
	OutTorque = (InClutchTorque + P2MotorTorque) * CurrentGearRatio * Config.Efficiency;
	//OutReflectedInertia = Config.InputShaftInertia * CurrentGearRatio * CurrentGearRatio;
}

void UVehicleGearboxComponent::UpdateInputShaft(
	float InAxleVelocity, 
	float InAxleInertia, 
	float InDriveShaftStiffness,
	float& OutClutchVelocity, 
	float& OutReflectedInertia,
	float& OutReflectedDriveShaftStiffness,
	float& OutCurrentGearRatio
)
{
	OutClutchVelocity = InAxleVelocity * CurrentGearRatio;
	// [previs AM-5 instrumentation patch] cache shaft speeds for read-only exposure
	LastInputShaftSpeed = OutClutchVelocity;
	LastOutputShaftSpeed = InAxleVelocity;
	const float GearRatioSquareInv = UVehicleUtilities::SafeDivide(1.f, CurrentGearRatio * CurrentGearRatio);
	OutReflectedInertia = InAxleInertia * GearRatioSquareInv;
	OutReflectedDriveShaftStiffness = InDriveShaftStiffness * GearRatioSquareInv;
	OutCurrentGearRatio = CurrentGearRatio;		
}

float UVehicleGearboxComponent::GetGearRatio(int InTarget)
{
	//InTarget = FMath::Clamp(InTarget, -ReverseGearRatios.Num(), GearRatios.Num());
	if (InTarget > Config.NumberOfGears || InTarget < -Config.NumOfReverseGears)return 0;
	if (InTarget > 0)return GearRatios[InTarget - 1];
	if (InTarget < 0)return ReverseGearRatios[-InTarget - 1];
	return 0.0f;
}

void UVehicleGearboxComponent::CalculateSpeedRangeOfEachGear(
	float InEffectiveWheelRadius,
	float InEngineIdleRPM, 
	float InEngineMaxRPM, 
	TArray<FVector2D>& OutSpeedRanges
)
{
	if (IsGearDataDirty())CalculateGearRatios();

	int32 NumGears = FMath::Max(Config.NumberOfGears, Config.NumOfReverseGears);
	OutSpeedRanges.SetNum(NumGears + 1);
	OutSpeedRanges[0] = FVector2D(0);

	float RPMToRad = PI * 0.0333333333333f;
	float avgRPM = InEffectiveWheelRadius * RPMToRad * 0.01;

	for (int32 i = 1; i <= NumGears; i++)
	{
		FVector2D SpeedRangeOfCurrentGear;
		float GearRatio = GetGearRatio(i);
		SpeedRangeOfCurrentGear.X = UVehicleUtilities::SafeDivide(InEngineIdleRPM * avgRPM, GearRatio);
		SpeedRangeOfCurrentGear.Y = UVehicleUtilities::SafeDivide(InEngineMaxRPM * avgRPM, GearRatio);

		OutSpeedRanges[i] = SpeedRangeOfCurrentGear;
	}
}

bool UVehicleGearboxComponent::CalculateGearRatios()
{
	if (Config.NumberOfGears < 1 || Config.NumOfReverseGears < 1)return false;

	//make sure positive
	Config.FirstGear = FMath::Abs(Config.FirstGear);
	Config.TopGear = FMath::Abs(Config.TopGear);
	Config.GearRatioBias = FMath::Clamp(FMath::Abs(Config.GearRatioBias), 0.f, 1.f);
	//make sure topgear is smaller than first gear
	Config.TopGear = FMath::Min(Config.TopGear, Config.FirstGear);

	//clear all gears
	GearRatios.SetNum(Config.NumberOfGears);
	ReverseGearRatios.SetNum(Config.NumOfReverseGears);

	if (Config.NumberOfGears >= Config.NumOfReverseGears)
	{
		return CalculateGearRatios(GearRatios, ReverseGearRatios);
	}
	else
	{
		return CalculateGearRatios(ReverseGearRatios, GearRatios, true);
	}
}

bool UVehicleGearboxComponent::CalculateGearRatios(TArray<float>& LargerArray, TArray<float>& SmallerArray, bool bInverseSign)
{
	int32 LargerLength = LargerArray.Num();
	int32 SmallerLength = SmallerArray.Num();

	if (LargerLength < SmallerLength)return false;

	float sign = 1;
	if (bInverseSign)sign = -1;

	if (LargerLength > 1)
	{
		for (int i = 0; i < LargerLength; ++i)
		{
			float t = (float)i / (float)(LargerLength - 1);	//normalize the factor, min = 0. max = 1
			//calculate bias
			float AdjustedT = FMath::Pow(t, 1.f - Config.GearRatioBias);
			LargerArray[i] = (Config.FirstGear + (Config.TopGear - Config.FirstGear) * AdjustedT) * sign;
		}
	}
	else if(LargerLength == 1)
	{
		LargerArray[0] = Config.FirstGear * sign;
	}
	else
	{
		return false;
	}

	for (int i = 0; i < SmallerLength; ++i)
	{
		SmallerArray[i] = -LargerArray[i];
	}

	return true;
}

bool UVehicleGearboxComponent::SetExplicitGearRatios(const TArray<float>& ForwardRatios, const TArray<float>& ReverseRatios)
{
	// The config stays exactly what SetConfig wrote (a read-back audit compares it); only the
	// ratio arrays change, and only when the source states one ratio per configured gear.
	if (ForwardRatios.Num() != Config.NumberOfGears || ReverseRatios.Num() != Config.NumOfReverseGears) return false;
	//: F1 (Codex review, 2026-09-10). The line here used to be `CalculateGearRatios()` with a comment
	//: claiming it refreshed the dirty-check cache. IT DOES NOT: only IsGearDataDirty() writes
	//: CachedFirstGear and its siblings, and on a fresh component those are -1. So the first shift
	//: request or speed-range calculation found the data dirty and SYNTHESISED the ratios back over
	//: the source's own. MEASURED on the E30, whose package states 3.72/2.40/1.77/1.26/1.00:
	//:   Previs.F1: gear ratios RECOMPUTED on a shift:
	//:     [3.720 2.400 1.770 1.260 1.000] -> [3.720 2.360 1.797 1.364 1.000]
	//: - fourth gear 8 % wrong, on every car, from its first shift onward.
	//:
	//: IsGearDataDirty() is called for its SIDE EFFECT: it copies the current config into the cache,
	//: so the arrays installed below are not recomputed away. FirstGear, TopGear and GearRatioBias
	//: only ever feed the synthesiser, and explicit ratios make them irrelevant; a change to the GEAR
	//: COUNTS does still invalidate them, which is correct, because a five-gear array cannot serve a
	//: six-gear box - and SetExplicitGearRatios refuses that mismatch at the top of this function.
	CalculateGearRatios();	//: sizes the arrays, as before
	IsGearDataDirty();		//: FOR THE SIDE EFFECT - primes the cache so the explicit arrays stand
	bHasExplicitGearRatios = true;
	GearRatios = ForwardRatios;
	for (float& R : GearRatios) R = FMath::Abs(R);
	ReverseGearRatios = ReverseRatios;
	for (float& R : ReverseGearRatios) R = -FMath::Abs(R);
	return true;
}

bool UVehicleGearboxComponent::IsGearDataDirty()
{
	//check if gearratios has to be calculated again
	if (CachedFirstGear != Config.FirstGear || CachedTopGear != Config.TopGear || CachedGearRatioBias != Config.GearRatioBias
		|| CachedNumGears != Config.NumberOfGears || CachedNumRGears != Config.NumOfReverseGears)
	{
		CachedFirstGear = Config.FirstGear;
		CachedTopGear = Config.TopGear;
		CachedGearRatioBias = Config.GearRatioBias;
		CachedNumGears = Config.NumberOfGears;
		CachedNumRGears = Config.NumOfReverseGears;
		return true;
	}
	else
	{
		return false;
	}
}