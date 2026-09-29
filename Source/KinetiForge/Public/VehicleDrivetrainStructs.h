// AWAITING FABLE REVIEW - changed 2026-09-10 by Opus 5 (CLAUDE.local.md section 1, ADR-041): FVehicleLimitedSlipDifferentialConfig::PreloadTorque, new.
// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "VehicleDrivetrainStructs.generated.h"

/*****************************ENGINE******************************/

UENUM(BlueprintType)
enum class EVehicleEngineOperationMode : uint8
{
	On  UMETA(DisplayName = "EngineON"),
	Off UMETA(DisplayName = "EngineOFF"),
	Starting    UMETA(DisplayName = "EngineSTARTING"),
	Shutting    UMETA(DisplayName = "EngineSHUTTING")
};

USTRUCT(BlueprintType, Blueprintable)
struct KINETIFORGE_API FVehicleNaturallyAspiratedEngineConfig
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float MaxEngineTorque = 400.f;

	/**
	* normalized engine torque curve
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	UCurveFloat* EngineTorqueCurve = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float EngineIdleRPM = 900.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float EngineMaxRPM = 6000.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float EngineInertia = 0.2f;

	/**
	* The internal friction of the engine at 0 rpm
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float StartFriction = 50.f;

	/**
	* The Slope (Tangent) of the internal friction of engine
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float FrictionCoefficient = 0.005f;

	/**
	* The Slope (Tangent) of the friction of engine caused by pumping
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0"))
	float PumpingLossCoefficient = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float IdleThrottleInterpSpeed = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float RevLimiterTime = 0.05f;

	/**
	* Previs 2026-09-30 (AWAITING FABLE REVIEW; the owner's throttle balance, Granddaddy Fable's approval): a drive-by-wire
	* car's PEDAL MAP - pedal (x, 0..1) to the throttle the engine is given (y, 0..1), piecewise linear, sorted by x.
	* A torque-demand ECU opens the throttle so the torque follows the pedal; without it a turbo car's pedal answers
	* throttle x boost(throttle), which puts most of the torque in the top half of the travel. Empty = the pedal IS the
	* throttle (every car before this existed, and every mechanical-throttle car).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (AdvancedDisplay))
	TArray<FVector2f> PedalToThrottle;

	/**
	* If Lambda < 1.f, there will be unburnt fuel, which will cause back fireing
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float MaxPowerLambda = 0.85f;

	/**
	* If Lambda < 1.f, there will be unburnt fuel, which will cause back fireing
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float DeceleratingLambda = 0.9f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EngineSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float UnburntFuelAccumulationRate = 2.f;
};

USTRUCT(BlueprintType, Blueprintable)
struct KINETIFORGE_API FVehicleEngineTurboConfig
{
	GENERATED_USTRUCT_BODY()

	/**
	* if max boost pressure > 0, will be considered as turbo charged
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float MaxBoostPressure = 0.f;

	/**
	* Minimum pressure (vacuum, restriction from the intake) when throttle input is really small(but above 0).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "-1.0", ClampMax = "0.0"))
	float StaticIntakeRestriction = -0.2f;

	/**
	* The engine RPM at which the turbo starts to generate positive pressure (spool up).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float SpoolStartRPM = 1200.f;

	/**
	* The engine RPM at which the turbo reaches maximum pressure.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float FullBoostRPM = 3500.f;

	/**
	* Time in seconds to reach ~95% max boost.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float SpoolUpDuration = 0.5f;

	/**
	* Time in seconds to loose pressure when throttle is closed(BOV open).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float PressureDecayDuration = 0.2f;

	/**
	* Determines how effectively boost pressure converts to torque.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoostEfficiency = 0.7f;

	/**
	* Enables Anti-Lag System (bang-bang) to keep turbo spooled off-throttle.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup")
	bool bEnableAntiLag = false;

	/**
	* minimum rpm to trigger anti-lag system
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float AntiLagMinRPM = 2000.f;

	/**
	* When antilag is activated, attempt to maintain a proportion of the maximum boost value. 
	* e.g. MaxBoostPressure = 1.0bar, AntiLagTargetPressureRatio = 0.8, then anti-lag system will keep the target pressure at 0.8bar.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TurboSetup", meta = (ClampMin = "0.0"))
	float AntiLagTargetPressureRatio = 0.9f;
};

USTRUCT(BlueprintType, Blueprintable)
struct KINETIFORGE_API FVehicleEngineExhaustConfig
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float ExhaustScavengingStrength = 2.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float HeatUpRate = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float CoolDownRate = 0.2f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", ClampMax = "1.0", AdvancedDisplay))
	float FlashPoint = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", ClampMax = "1.0", AdvancedDisplay))
	float IgnitionProbability = 0.1f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float PopFuelThreshold = 0.2f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float FlameFuelThreshold = 0.25f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ExhaustSetup", meta = (ClampMin = "0.0", AdvancedDisplay))
	float BackfireHeatSpike = 0.15f;
};

USTRUCT(BlueprintType, Blueprintable)
struct KINETIFORGE_API FVehicleEngineSimState
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	EVehicleEngineOperationMode OperationMode = EVehicleEngineOperationMode::Starting;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fuel")
	bool bSpark = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fuel")
	bool bFuelInjection = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float EngineAngularVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Input")
	float RawThrottleInput = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Input")
	float IdleThrottle = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Input")
	float RealThrottle = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float EffectiveTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float LoadTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float TorqueRequiredToStartEngine = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float StarterMotorTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float EngineRPM = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float EngineOffRPM = 0.f;	//under this rpm, the engine will be considered as off
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fuel")
	float UnburntFuelBuffer = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turbo")
	float TurboSpool = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turbo")
	float TurboPressure = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Exhaust")
	float ExhaustHeat = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Exhaust")
	float BackfireIntensity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float RevLimiterTimer = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float P1MotorTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turbo")
	bool bIsTurboBlowingOff = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Turbo")
	bool bIsAntiLagTriggered = false;
};

/*******************************CLUTCH********************************/


UENUM(BlueprintType)
enum class EClutchSimMode : uint8
{
	/**
	 * Simulates a friction clutch with torsional compliance.
	 * The clutch stiffness is combined in series with the reflected
	 * stiffness of the downstream driveline.
	 */
	FrictionClutch UMETA(DisplayName = "Friction Clutch"),

	/**
	 * Simulates a fluid coupling / torque converter as a viscous damper.
	 * No torsional spring state is accumulated.
	 */
	FluidCoupling UMETA(DisplayName = "Fluid Coupling"),

	/**
	 * Uses impulse analysis to calculate the torque required to reduce
	 * the relative angular velocity within the current physics step.
	 */
	ConstraintLock UMETA(DisplayName = "Constraint Lock")
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleClutchConfig
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup")
	EClutchSimMode SimMode = EClutchSimMode::FrictionClutch;

	/**
	 * Torsional stiffness of the clutch/crank/input-side elastic path.
	 * Used by FrictionClutch.
	 * Unit: Nm/Rad
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup",
		meta = (ClampMin = "0.0", EditCondition = "SimMode == EClutchSimMode::FrictionClutch", EditConditionHides))
	float TorsionalStiffness = 10000.f;

	/**
	 * Damping ratio of the equivalent torsional spring mode.
	 * Used by FrictionClutch.
	 * 0 = no damping, 1 = critical damping.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup",
		meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "SimMode == EClutchSimMode::FrictionClutch", EditConditionHides))
	float DampingRatio = 0.1f;

	/**
	 * Viscous damping coefficient of the fluid coupling.
	 * Used by FluidCoupling.
	 * Unit: Nm*s/Rad
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup",
		meta = (ClampMin = "0.0", EditCondition = "SimMode == EClutchSimMode::FluidCoupling", EditConditionHides))
	float ViscousDamping = 100.f;

	/**
	 * Maximum transmissible clutch torque multiplier.
	 *
	 * Real capacity:
	 * Capacity * MaxEngineTorque, including boost multiplier.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup", meta = (ClampMin = "0.0"))
	float Capacity = 1.5f;

	/**
	 * Previs, 2026-09-25 (the owner: "make sure clutch has realistic slip grip"): kinetic over static
	 * friction of the facing. A SLIPPING clutch carries Capacity x KineticRatio; it breaks away when
	 * the torque asked of it passes the static capacity and re-locks when the kinetic capacity can
	 * close the slip. 1.0 = one Coulomb capacity both ways, the behaviour before this field.
	 * Used by FrictionClutch and ConstraintLock (a fluid coupling has no static friction).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ClutchSetup", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float KineticRatio = 1.f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleClutchSimState
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float AngleDiff = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float ClutchLock = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float MaxClutchTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float ClutchTorque = 0.f;
	/** Previs, 2026-09-25: the facing is slipping (kinetic capacity) rather than stuck (static). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	bool bSlipping = false;
	/** Previs, 2026-09-25: the sign of the facings' slip when they broke away; they stick again where it changes. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float SlipSign = 0.f;
};

/*******************************GEARBOX********************************/


USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleGearboxConfig
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float ShiftDelay = 0.2;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float FirstGear = 3.636;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float TopGear = 0.842;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float GearRatioBias = 0.5;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "1.0"))
	int32 NumberOfGears = 6;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "1.0", ToolTip = ""))
	int32 NumOfReverseGears = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Efficiency = 0.9;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	bool bSequentialGearbox = false;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FAutoGearboxConfig
{
	GENERATED_USTRUCT_BODY()

	/*
	* This simulates the logic of a real automatic gearbox depending on the vehicle speed and the throttle/brake input. 
	* This is actually more like a AMT gearbox.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAutomaticGearbox = true;

	/*
	* this helps shifting from D to R / R to D / N to D / D to N / N to R / R to N 
	* (automatic gearboxes in reallife will never do this)
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bArcadeAutoGearbox = true;

	/*
	* if true, the arcade gearbox will be updated without any delay. 
	* eg. shift from D to N immediately, then in the next frame, shift to D again immediately.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bArcadeShiftInstant = false;

	/*
	* if using sport mode, the auto gearbox will shift up as late as possible and shift down as quick as possible. 
	* Also, automatic rev-matching."
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bSportMode = false;

	/*
	* this roughly decides the refershrate of the auto gearbox. 
	* eg. if AutoGearboxRefreshTime = 0.5 seconds, the auto gearbox will be refreshed 2 times per second.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float AutomaticGearboxRefreshTime = 0.5;

	/*
	* this roughly decides how long the auto gearbox needs to cool down after one shift.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float AutoShiftCoolDown = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 MaxUpShiftSteps = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 MaxDownShiftSteps = 2;

	/*
	* At which RPM (normalized) the gearbox will shift up.
	* This should be greater than DownShiftRPM.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "0.99"))
	float UpShiftRPM = 0.95f;

	/*
	* At which RPM (normalized) the gearbox will shift down.
	* This should be smaller than UpShiftRPM
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "0.99"))
	float DownShiftRPM = 0.9f;

	/*
	* This curve scales the UpShiftRPM.
	* The input will be Max(Throttle, Brake).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = ""))
	UCurveFloat* UpShiftRPMCurve = nullptr;

	/*
	* This curve scales the DownShiftRPM.
	* The input will be Max(Throttle, Brake).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ToolTip = ""))
	UCurveFloat* DownShiftRPMCurve = nullptr;
};

/**************************DIFFERENTIAL*****************************/

USTRUCT(BlueprintType)
struct FVehicleLimitedSlipDifferentialConfig
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float GearRatio = 3.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float DriveLockRatio = 0.f;	//range: 0 - 1
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float CoastLockRatio = 0.f;	//range: 0 - 1

	/**
	* Clutch-pack PRELOAD, N*m: the torque the pack transfers with no input torque at all, which is
	* what holds a limited-slip together off the throttle. Added 2026-09-10 (Codex review F4, and the
	* owner: "more cars than the quattro will need a preloaded diff") - 165 of Drive's 213 cars state
	* one, median 10 N*m and up to 300 on the Sport Quattro S1 E2's centre differential, and every one
	* of them was being discarded because this model had nowhere to put it.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float PreloadTorque = 0.f;	//N*m

	/**
	* VISCOUS COUPLING coefficient, N*m*s/rad (Drive 2026-09-24, the Audi V8 quattro DTM's centre and
	* any car whose research states one). A viscous coupling transfers a torque proportional to the
	* speed difference across it - c x |omega difference| - with no torque-sensitive part and no
	* preload. It joins the pack's capacity:
	*
	*     capacity = preload + lock_ratio x |input torque| + c x |omega difference|
	*
	* so a pure coupling states a coefficient with zero lock and zero preload, and a plate-and-viscous
	* unit states both. A differential stating a coefficient uses the equalising torque as its raw
	* (the axle law's; the centre law's raw was lock x that, which is zero for a coupling with no
	* plates) and the capacity bounds it. Zero, the default, changes nothing for every car that
	* states none.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float ViscousCoefficient = 0.f;	//N*m*s/rad

	/**
	* THE ON-DEMAND COUPLING (Drive 2026-09-25, the owner: "simulate it and create a new component if
	* needed"): a transfer case that is NOT a differential but a controlled clutch - Audi's R8, Lamborghini's
	* Haldex, Porsche's PTM, Nissan's ATTESA - as Assetto Corsa's [AWD2] computes it (acs.exe
	* Drivetrain::step4WD_new, docs/code reference/physics/ac-awd2-clutch-law-2026-09-25.md): the drive goes
	* to the PRIMARY axle(s) (TorqueWeight > 0) only; each SECONDARY axle (TorqueWeight 0) is fed by the
	* clutch alone, T = clamp(CouplingRampTorque x (w_primary - w_secondary), +-CouplingMaxTorque), the
	* reaction taken off the primary. Both values are at this node (the propshaft): the harness converts
	* the wheel-level law. CouplingMaxTorque is set each frame by a controller (the clutch's command).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	bool bOnDemandCoupling = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float CouplingRampTorque = 0.f;	//N*m per rad/s of speed difference, at this node
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.0"))
	float CouplingMaxTorque = 0.f;	//N*m, at this node
	/* ONE-WAY (Previs, 2026-09-25; AWAITING FABLE REVIEW): an overrunning clutch (Polaris' Hilliard front gearcase)
	* transfers only while the primary side runs ahead of the secondary in the primary's own direction of
	* rotation, and freewheels otherwise. With a secondary geared shorter than the primary (the MRZR: front 3.82,
	* rear 3.53) it engages only past the ratio mismatch in rear slip (~8 %), as the real one does.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	bool bOneWayCoupling = false;
	/* SECONDARY SPEED SCALE (Previs, 2026-09-29; AWAITING FABLE REVIEW; VD's review MED of the commanded clutch): the
	* coupling compares w_primary - scale x w_secondary. A controller that reads ROAD speeds (a PTM or Haldex unit) sets
	* scale = (r_secondary / r_primary) x (FD_primary / FD_secondary), so equal road speeds read zero and a pre-engaged
	* clutch clamps from zero slip; with unequal tyres at 1.0 the smaller-tyred axle's shaft runs ahead and a one-way
	* clutch freewheels until the slip overtakes the mismatch. 1.0 = the plain propshaft comparison (the default).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (ClampMin = "0.5", ClampMax = "2.0"))
	float CouplingSecondarySpeedScale = 1.f;
};
