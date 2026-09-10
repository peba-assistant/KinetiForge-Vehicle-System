// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "VehicleDrivetrainStructs.h"
#include "VehicleGearboxComponent.generated.h"

DECLARE_DYNAMIC_DELEGATE(FOnShiftFinishedDelegate);

class UVehicleAxleAssemblyComponent;

// [previs AM-5 instrumentation patch] Read-only snapshot of the gearbox shift machine, taken in
// one call so a reader outside the physics window cannot tear across five separate getters.
// Adds no behaviour: every field is assembled from state the gearbox already keeps.
struct FVehicleGearboxShiftState
{
	int32 CurrentGear = 0;		// during a shift this is still the START gear; FinalizeShift moves it
	int32 TargetGear = 0;		// 0 when no shift is in flight
	int32 Direction = 0;		// +1 up, -1 down, 0 not shifting
	bool bInProgress = false;	// !bIsInGear
	bool bSparkCut = false;		// sequential upshift torque cut window, read not estimated
	bool bRevMatch = false;		// downshift blip window, read not estimated
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent), BlueprintType, Blueprintable)
class KINETIFORGE_API UVehicleGearboxComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UVehicleGearboxComponent();

protected:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Setup")
	FVehicleGearboxConfig Config;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	FOnShiftFinishedDelegate ShiftFinishedCallback;
	FTimerHandle GearShiftTimerHandle;
	TArray<float> GearRatios;
	TArray<float> ReverseGearRatios;
	float CurrentGearRatio;
	float P2MotorTorque = 0.f;
	int32 CurrentGear;
	int32 TargetGear;
	bool bIsInGear = true;
	bool bShouldRevMatch = false;
	bool bShouldCutSpark = false;

	//cache
	//: F1 (2026-09-10): the source stated its own ratios, so the synthesiser must not run again.
	bool bHasExplicitGearRatios = false;
	float CachedFirstGear = -1;
	float CachedTopGear = -1;
	float CachedGearRatioBias = -1;
	int32 CachedNumGears = -1;
	int32 CachedNumRGears = -1;

	void StartShift(int32 InTargetGear, bool bImmediate = false);
	void FinalizeShift();

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	const FVehicleGearboxConfig& GetConfig() { return Config; }

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void SetConfig(const FVehicleGearboxConfig& NewConfig) { Config = NewConfig; }

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void ShiftToTargetGear(int32 InTargetGear, bool bImmediate = false);

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void ShiftToTargetGearWithDelegate(FOnShiftFinishedDelegate InOnShiftFinished, int32 InTargetGear, bool bImmediate = false);

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void UpdateOutputShaft(
		float InClutchTorque, 
		float& OutTorque
	);

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void UpdateInputShaft(
		float InAxleVelocity,
		float InAxleInertia,
		float InDriveShaftStiffness,
		float& OutClutchVelocity,
		float& OutReflectedInertia,
		float& OutReflectedDriveShaftStiffness,
		float& OutCurrentGearRatio
	);

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	float GetGearRatio(int InTarget);

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	int32 GetCurrentGear() { return CurrentGear; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	float GetCurrentGearRatio() { return CurrentGearRatio; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	bool GetIsInGear() { return bIsInGear; }

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	void SetP2MotorTorque(float NewTorque) { P2MotorTorque = NewTorque; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	float GetP2MotorTorque() { return P2MotorTorque; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VehicleGearbox")
	void CalculateSpeedRangeOfEachGear(
		float InEffectiveWheelRadius,
		float InEngineIdleRPM,
		float InEngineMaxRPM,
		TArray<FVector2D>& OutSpeedRanges
	);

	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox", meta = (ToolTip = "Calculate and update gear ratios"))
	bool CalculateGearRatios();
	bool CalculateGearRatios(TArray<float>& LargerArray, TArray<float>& SmallerArray, bool bInverseSign = false);

	bool IsGearDataDirty();
	//: Previs (2026-09-02): the source's own per-gear ratios, exact. KinetiForge synthesises the
	//: intermediates from first/top/bias; a vehicle that states every ratio hands them over here,
	//: after SetConfig, and the cached first/top/bias/count keep IsGearDataDirty() false.
	UFUNCTION(BlueprintCallable, Category = "VehicleGearbox")
	bool SetExplicitGearRatios(const TArray<float>& ForwardRatios, const TArray<float>& ReverseRatios);
	bool GetShouldRevMatch() { return bShouldRevMatch; }
	bool GetShouldCutSpark() { return bShouldCutSpark; }

	// [previs AM-5 instrumentation patch] Read-only exposure; no behaviour change.
	int32 GetTargetGear() { return TargetGear; }
	float GetInputShaftSpeed() { return LastInputShaftSpeed; }		// clutch side, rad/s
	float GetOutputShaftSpeed() { return LastOutputShaftSpeed; }	// axle side, rad/s
	FVehicleGearboxShiftState GetShiftState()
	{
		FVehicleGearboxShiftState State;
		State.CurrentGear = CurrentGear;
		State.TargetGear = TargetGear;
		State.bInProgress = !bIsInGear;
		if (!bIsInGear)
		{
			State.Direction = FMath::Abs(TargetGear) > FMath::Abs(CurrentGear) ? 1 : -1;
		}
		State.bSparkCut = bShouldCutSpark;
		State.bRevMatch = bShouldRevMatch;
		return State;
	}

private:
	// [previs AM-5 instrumentation patch] Cached by UpdateInputShaft each call; read-only outside.
	float LastInputShaftSpeed = 0.f;
	float LastOutputShaftSpeed = 0.f;
};
