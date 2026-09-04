// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "VehicleUtilities.h"
#include "VehicleWheelStructs.generated.h"

UENUM(BlueprintType)
enum class ETireFrictionCombineMode : uint8
{
	Constant,
	Average,
	Multiply,
	Min,
	Max
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleWheelConfig
{
	GENERATED_BODY()

	/*Unit: cm*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Radius = 33.f;

	/*Unit: cm*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Width = 20.f;

	/*Unit: kg*m^2*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Inertia = 1.f;

	/**
	* This is just a constant additional handbrake torque
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float RollingRisistance = 5.f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleTireConfig
{
	GENERATED_BODY()

	/**
	* Overall grip multiplier for this tire.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Global Friction Scale"))
	float FrictionMultiplier = 1.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ETireFrictionCombineMode TireFrictionCombineMode = ETireFrictionCombineMode::Average;

	/**
	* Simulates tire lag. Higher values = softer/laggy response.
	* Increase this if the vehicle jitters at low frame rates.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Relaxation Length"))
	FVector2f RelaxationLength = FVector2f(0.1f, 0.2f);

	/**
	* if value is 1, the wheel load will be proportional to suspension force. 
	* If value is 0, the wheel load will be constant (gravity of sprungmass).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WheelLoadInfluenceFactor = 0.8f;

	/**
	* TYRE LOAD SENSITIVITY, in PROJECT CHRONO'S OWN SHAPE (ADR-031, 2026-09-04).
	*
	* A tyre's friction coefficient falls as its vertical load rises. Chrono is the larger and more
	* precise model, so its implementation is the one this extension carries, and every source is
	* mapped INTO it rather than each bringing its own curve (owner, 2026-09-04: "let chrono decide
	* how the implementation is done, so we have the larger more precise model as reference point").
	* One shape means a tyre is the same two numbers in both simulators, which is what makes a car
	* replicable across them.
	*
	* Chrono (ChTMeasyTire) states a tyre's peak force at a nominal load pn and again at 2 pn, and
	* interpolates quadratically between them through the origin (its InterpQ):
	*     q     = clamp(Fz, 0, 3.5 pn) / pn
	*     scale = q * (2 - r/2 - (1 - r/2) * q)      with r = force(2pn) / force(pn)
	*     force = mu_ref * pn * scale
	* which is exactly force(pn) at q = 1 and force(2pn) at q = 2, and 0 at no load. The 3.5 pn
	* clamp is Chrono's own pn_max. The Fx/Fy curve already carries mu_ref as its peak, so what
	* lives here is the load term alone.
	*
	* r is the whole of the load sensitivity in one number. r = 2 is a tyre that never loses grip
	* with load; every real tyre is below it. Chrono's own truck tyre is 1.914 laterally, its
	* passenger tyre 1.826. Assetto states the same physics as a power law (FZ0 with LS_EXPX/LS_EXPY)
	* and converts exactly: r = 2^LS_EXP, which for the E30 M3's 0.7351 is 1.665 - a road tyre that
	* loses grip with load faster than either of Chrono's, which is what a soft road compound does.
	*
	* ReferenceLoad <= 0 or a ratio <= 0 leaves the saturating fallback in CalculateAvailableGrip in
	* charge, which is what a source stating no load law gets.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Load Sensitivity: Reference Load (N)"))
	float LoadSensitivityReferenceLoad = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "2.0", DisplayName = "Load Sensitivity: Force Ratio at Double Load (Longitudinal)"))
	float LoadForceRatioAtDoubleLoadLong = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "2.0", DisplayName = "Load Sensitivity: Force Ratio at Double Load (Lateral)"))
	float LoadForceRatioAtDoubleLoadLat = 0.f;

	/**
	* Balances grip between turning and accelerating.
	* 0.5 = Balanced.
	* >0.5 = Prioritizes Turning (Easier to corner while braking).
	* <0.5 = Prioritizes Acceleration.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Grip Distribution Bias"))
	float CombinedSlipBias = 0.5f;

	/**
	* How much longitudinal slip (spinning) reduces lateral grip.
	* 1.0 = Realistic (Spinning wheels can't turn).
	* 0.0 = Arcade (Full turning grip even when burning out).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Accel Affects Turning"))
	float LongitudinalToLateralInterference = 1.f;

	/**
	* How much lateral slip (drifting) reduces forward acceleration.
	* Lower values (e.g., 0.8) make it easier to maintain speed while drifting.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Turning Affects Accel"))
	float LateralToLongitudinalInterference = 1.f;

	/**
	* Scales the Fx curve output. Determines max longitudinal force.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Longitudinal Grip Limit"))
	float MaxFx = 1.f;

	/**
	* Scales the Fy curve output. Determines max cornering force.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Lateral Grip Limit"))
	float MaxFy = 1.f;

	/**
	* Input: Slip Ratio. Output: Friction Coefficient.
	* Note: Slope at origin will be used to determine Optimal Slip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Fx (Longitudinal)"))
	UCurveFloat* Fx = nullptr;

	/**
	* Input: Slip Angle (Deg). Output: Friction Coefficient.
	* Note: Slope at origin will be used to determine Optimal Slip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Fy (Lateral)"))
	UCurveFloat* Fy = nullptr;

	/**
	* Input: abs camber angle in degrees.
	* Output: lateral drift per rolling distance, dy / dx.
	*
	* This is not a force scale.
	* This is not a constant lateral velocity.
	*
	* The solver converts it to lateral slip velocity:
	*
	*     CamberSlipVelocityY = RollSpeed * CamberLateralDrift
	*
	* Example:
	*     Camber = 5 deg
	*     Curve output = 0.01
	*     means the contact patch tends to drift 1 cm laterally per 1 m rolling distance.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Camber To Lateral Drift"))
	UCurveFloat* CamberToLateralDrift = nullptr;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleABSConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAntiBrakeSystemEnabled = true;

	/**
	* The target slip ratio the anti-brake-system try to maintain.
	* The value is dependent on your tire config and the road condition.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float OptimalSlip = 0.1f;

	/**
	* Minimum speed required to activate anti-brake-system.
	* Unit: meter per second, m/s
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float ActivationSpeed = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Sensitivity = 5.f;
};

template <int32 NumSamples>
struct KINETIFORGE_API FVehicleTireLUT : public FVehicleLUT<NumSamples>
{
public:
	// True, if can be used for combined slip
	bool bHasValidStiffness = false;

	int32 OptimalSlipIndex = 0;
	float LinearStiffness = 0.f;
	float PeakFriction = 0.f;

	FVehicleTireLUT(const float InitValue = 0.f) : FVehicleLUT<NumSamples>(InitValue)
	{
	}

	FVehicleTireLUT(const FRichCurve& RichCurve, const FVector2f SelectedTimeInterval = FVector2f(0.f, 1.f))
	{
		BuildFromCurve(RichCurve, SelectedTimeInterval);
	}

	void BuildFromCurve(const FRichCurve& RichCurve, const FVector2f SelectedTimeInterval = FVector2f(0.f, 1.f))
	{
		// Call the superclass method to copy the data and populate the `Samples` array
		this->CopyFromRichCurve(RichCurve, SelectedTimeInterval);

		bHasValidStiffness = false;
		LinearStiffness = 0.f;
		PeakFriction = 0.f;

		if (RichCurve.Keys.Num() == 0) return;

		const float TimeMin = SelectedTimeInterval.X;
		const float TimeMax = SelectedTimeInterval.Y;
		const float IntervalLength = TimeMax - TimeMin;

		if (IntervalLength <= SMALL_NUMBER) return;

		// Constrain the negative force and find the global maximum grip force
		for (int32 i = 0; i < NumSamples; i++)
		{
			if (this->Samples[i] < 0.f)
			{
				this->Samples[i] = 0.f; // Forcefully Compensate for Negative Grip
			}

			if (this->Samples[i] > PeakFriction)
			{
				PeakFriction = this->Samples[i];
			}
		}

		const float StartValue = this->Samples[0];
		const float EndValue = this->Samples[NumSamples - 1];

		// Finding the Optimal Slip Using the Maximum Deviation Method
		const float SecantSlope = (EndValue - StartValue) / (float)(NumSamples - 1);

		float MaxDeviation = -1.f;
		int32 OptimalLocalIndex = 0;

		for (int32 i = 0; i < NumSamples; i++)
		{
			float CurrentY = this->Samples[i];
			float SecantY = StartValue + SecantSlope * i;
			float Deviation = CurrentY - SecantY;

			if (Deviation > MaxDeviation)
			{
				MaxDeviation = Deviation;
				OptimalLocalIndex = i;
			}
		}

		// cache optimal slip
		OptimalSlipIndex = OptimalLocalIndex;

		// try to find stiffness of linear region
		float MaxLocalStiffness = 0.f;
		for (int32 i = 0; i < OptimalSlipIndex; i++)
		{
			float CurrentY = this->Samples[i];
			float NextY = this->Samples[i + 1];
			float LocalStiffness = (NextY - CurrentY) * (float)(NumSamples - 1);

			if (LocalStiffness > MaxLocalStiffness)
			{
				MaxLocalStiffness = LocalStiffness;
			}
		}

		LinearStiffness = MaxLocalStiffness;

		// Check if stiffness is valid
		if (LinearStiffness > SMALL_NUMBER)
		{
			bHasValidStiffness = true;
		}
	}
};

USTRUCT()
struct KINETIFORGE_API FVehicleWheelCachedLUTs
{
	GENERATED_BODY()

	FVehicleTireLUT<64> Fx = FVehicleTireLUT<64>(1.f);
	FVehicleTireLUT<64> Fy = FVehicleTireLUT<64>(1.f);
	FVehicleLUT<64> CamberToLateralDrift = FVehicleLUT<64>(0.f);
};

USTRUCT(BlueprintType, meta = (ToolTip = "wheel state in simulation"))
struct KINETIFORGE_API FVehicleWheelSimState
{
	GENERATED_BODY()
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float EffectiveInertia = 1.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float AngularVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float AngularAcceleration = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float LongSlipVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float LatSlipVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float SlipRatio = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float SlipAngle = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float PredictedSlipRatio = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float SignedCamberDegree = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float DynFrictionMultiplier = 1.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	FVector2f LocalLinearVelocity = FVector2f(0.f, 0.f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float WheelLoad = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float P4MotorTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float DriveTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorque = 0.f;		//brake torque from brake + rolling resistance
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromBrake = 0.f;	//brake torque input + brake torque from esp; not including the rolling resistance
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromHandbrake = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromESP = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float TorqueFromGroundInteraction = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	FVector2f TransientSlip = FVector2f(0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	FVector2f TireForce2D = FVector2f(0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	FVector3f TireForce = FVector3f(0.f, 0.f, 0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	bool bIsLocked = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Electronics")
	bool bABSTriggered = false;
};

USTRUCT(BlueprintType, meta = (ToolTip = ""))
struct KINETIFORGE_API FVehicleWheelSimContext
{
	GENERATED_BODY()

	float MacroDeltaTime = 1 / 120.f;
	float MacroDeltaTimeInv = 120.f;
	float SubstepDeltaTime = 1 / 120.f;
	float SubstepDeltaTimeInv = 120.f;

	float LongForceScale = 1.f;
	float LatForceScale = 1.f;
	FVector3f LongForceDir = FVector3f(0.f);
	FVector3f LatForceDir = FVector3f(0.f);

	float CamberLateralDrift = 0.f;

	float AvailableGrip = 0.f;
	//: ADR-031: the same with the LATERAL exponent. A tyre's two directions lose grip with load at
	//: different rates and the source states them apart (AC's LS_EXPX 0.8001 against LS_EXPY 0.7351).
	float AvailableGripLat = 0.f;
	FVector2f GravityComp2D = FVector2f(0.f);
	FVector2f AccumulateTireImpulse2D = FVector2f(0.f);

	float R = 0.33f;
	float RInv = 1 / 0.33f;
};