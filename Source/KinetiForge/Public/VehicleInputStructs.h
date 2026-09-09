// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "VehicleInputStructs.generated.h"

USTRUCT(BlueprintType)
struct FVehicleInputAxisConfig
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2f InterpSpeed = FVector2f(5.f, 5.f);
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UCurveFloat* ResponseCurve = nullptr;

    FVehicleInputAxisConfig(
        FVector2f newInterpSpeed = FVector2f(5.f, 5.f),
        UCurveFloat* newResponseCurve = nullptr)
    {
        InterpSpeed = newInterpSpeed;
        ResponseCurve = newResponseCurve;
    }

    static float InterpInputValueConstant(
        float Current,
        float Target,
        float DeltaTime,
        FVector2f Speed
    )
    {
        float s = (Target < SMALL_NUMBER) ? Speed.Y : Speed.X;
        return (s <= 0) ? Target : FMath::FInterpConstantTo(Current, Target, DeltaTime, s);
    }

    static float InterpInputValue(
        float Current,
        float Target,
        float DeltaTime,
        FVector2f Speed
    )
    {
        float s = (Target < SMALL_NUMBER) ? Speed.Y : Speed.X;
        return (s <= 0) ? Target : FMath::FInterpTo(Current, Target, DeltaTime, s);
    }
};

USTRUCT(BlueprintType)
struct FVehiclInputConfig
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputAxisConfig Throttle;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputAxisConfig Brake;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputAxisConfig Clutch;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputAxisConfig Handbrake = FVehicleInputAxisConfig(FVector2f(15.f, 15.f));
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputAxisConfig Steering = FVehicleInputAxisConfig(FVector2f(2.5f, 2.5f));
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UCurveFloat* HighSpeedSteeringScale = nullptr;
};

USTRUCT(BlueprintType)
struct FVehicleInputAssistConfig
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = "engage clutch when eg. changing gear or low rpm"))
    bool bAutomaticClutch = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = "disable AutomaticClutch, and disable throttle when in N gear"))
    bool bEVClutchLogic = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = "at which rpm the clutch should be (gradually) released"))
    FVector2f AutoClutchRange = FVector2f(1200, 2500);
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bRevMatching = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float RevMatchMaxThrottle = 0.6;
    //: **THE LAUNCH (Previs, owner 2026-09-10: an F40 at full throttle "roms sag to 1400 rpm and the
    //: car creeps away" where it should light its tyres).** `AutoClutchRange` closes the clutch AS
    //: THE ENGINE RISES, which on a turbo car is a trap: at 1436 rpm the band held 61 % of the
    //: clutch open and passed 124 N.m - about all the F40 makes off boost - so engine torque and
    //: clutch capacity climbed together and the engine could never get through the band. Measured
    //: in his own drive log. A driver launching a car does the opposite: he holds the engine at a
    //: launch speed on a slipping clutch and lets it bite there. So at deliberate throttle, from a
    //: low speed, in gear, the clutch is held OPEN below `LaunchRpm` and closes hard above it.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = "hold the clutch slipping to a launch rpm at full throttle from low speed, instead of closing it by rpm alone"))
    bool bLaunchClutch = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "throttle above which a launch is deliberate"))
    float LaunchThrottle = 0.7f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ToolTip = "m/s below which a launch clutch applies; above it the ordinary band does"))
    float LaunchSpeed = 8.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "where the launch rpm sits between the auto-clutch band's top and the limiter"))
    float LaunchRpmFraction = 0.45f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAutoHold = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = "Releases the brake of the axle if the torque weight(normalized) is > 0.5"))
    bool bBurnOutAssist = true;
};

USTRUCT(BlueprintType)
struct FVehicleInputState
{
    GENERATED_USTRUCT_BODY()

    //input
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Throttle = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Brake = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Clutch = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Handbrake = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Steering = 0.f;
};

USTRUCT(BlueprintType)
struct FVehicleInputPipeline
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputState Raw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputState Smoothened;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVehicleInputState Final;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bSwitchThrottleAndBrake = false;
};