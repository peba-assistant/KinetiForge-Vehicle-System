// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)


#include "VehicleWheelSolver.h"
#include "VehicleWheelComponent.h"
#include "AsyncTickFunctions.h"
#include "VehicleUtilities.h"

FVehicleWheelSolver::FVehicleWheelSolver()
{
}

FVehicleWheelSolver::~FVehicleWheelSolver()
{
}

void FVehicleWheelSolver::Initialize(const FVehicleTireConfig& TireConfig)
{
	UpdateCachedLUTs(TireConfig);
}

void FVehicleWheelSolver::PreStep(
	float InMacroDeltaTime,
	const FTransform& AsyncChassisWorldTransform,
	const FVehicleSuspensionSimState& SuspensionState, 
	const FVehicleWheelConfig& WheelConfig,
	const FVehicleTireConfig& TireConfig)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(KinetiForgeVehicle_Wheel_WheelSolver_UpdateWheel);

	FVehicleWheelSimContext& Context = CurrentContext;
	FVehicleWheelSimState& LocalState = State;

	Context.MacroDeltaTime = InMacroDeltaTime;
	Context.MacroDeltaTimeInv = UVehicleUtilities::SafeDivide(1.f, InMacroDeltaTime);

	Context.R = WheelConfig.Radius * 0.01f;
	Context.RInv = UVehicleUtilities::SafeDivide(1.f, Context.R);

	const FVector3f WheelRightVec = SuspensionState.WheelWorldRightVector;
	const FVector3f ImpactNormal = SuspensionState.ImpactWorldNormal;

	const FVector3f LongForceDirUnNorm = FVector3f::CrossProduct(WheelRightVec, ImpactNormal);
	const FVector3f LatForceDirUnNorm = FVector3f::VectorPlaneProject(WheelRightVec, ImpactNormal);
	Context.LongForceDir = LongForceDirUnNorm.GetSafeNormal();
	Context.LatForceDir = LatForceDirUnNorm.GetSafeNormal();

	// ÕâÀïµÄµã³ËÆäÊµ¾ÍÊÇ»ñÈ¡ÏòÁ¿³¤¶È
	// ÎÒÊ¹ÓÃµã³ËÖ»ÊÇÎªÁËÌáÐÑ×Ô¼º£¬ÎÒÔÚÓÃ³µÂÖ×ªÖá·½ÏòÔÚµØÃæÉÏµÄÍ¶Ó°³¤¶ÈÀ´½üËÆÄ£ÄâcamberÔì³ÉµÄ×¥µØÁ¦±ä»¯
	Context.LongForceScale = 1.f;
	Context.LatForceScale = 1.f;

	UpdateLinearVelocity(LocalState, Context.LongForceDir, Context.LatForceDir, SuspensionState.ImpactWorldVelocity);

	UpdateDynamicFrictionMultiplier(LocalState, Context, TireConfig, SuspensionState.ImpactFriction);

	LocalState.WheelLoad = FMath::Max(0.f, SuspensionState.ForceAlongImpactNormal);

	Context.GravityComp2D = CalculateGravityCompensationOnSlope(
		LocalState, Context,
		LocalState.WheelLoad,
		SuspensionState.bWheelOnGround,
		Context.LongForceDir,
		Context.LatForceDir
	);

	// One road-relative camber and one pressure snapshot for this macro step.
    const FVector3f W = WheelRightVec.GetSafeNormal(), N = ImpactNormal.GetSafeNormal();
    const bool ContactValid = SuspensionState.bWheelOnGround && !Context.LongForceDir.IsNearlyZero()
        && !Context.LatForceDir.IsNearlyZero();
    LocalState.RawCamberRad = ContactValid ? FMath::Asin(FMath::Clamp(FVector3f::DotProduct(W,N),-1.f,1.f)) : 0;
    LocalState.SignedCamberDegree = FMath::RadiansToDegrees(LocalState.RawCamberRad) * (SuspensionState.bIsRightWheel?-1:1);
    const auto& P = TireConfig.ResponseProfile;
    Context.Response = previs::tire::response::evaluate(P,TireConfig.OperatingPressurePa,
        ContactValid ? LocalState.WheelLoad : 0,LocalState.RawCamberRad);
    LocalState.PressurePa=TireConfig.OperatingPressurePa;
    LocalState.bResponseValid=Context.Response.valid;
    LocalState.bResponseExtrapolated=Context.Response.extrapolated;
    const auto Grip = [&](float Mu,float Ratio,float Load) {
        return CalculateAvailableGrip(Mu,SuspensionState.StaticSprungMass,Load,TireConfig.WheelLoadInfluenceFactor,
            TireConfig.LoadSensitivityReferenceLoad,Ratio);
    };
    const float G0=FMath::Max(CachedLUTs.CamberToGripFactor.FastEval(0).Value,SMALL_NUMBER);
    const float Gy=FMath::Max(0.f,CachedLUTs.CamberToGripFactor.FastEval(FMath::Abs(Context.Response.gamma)/(PI/2)).Value/G0);
    Context.AvailableGrip=Grip(LocalState.DynFrictionMultiplier,TireConfig.LoadForceRatioAtDoubleLoadLong,LocalState.WheelLoad)*Context.Response.grip_x;
    Context.AvailableGripLat=Grip(LocalState.DynFrictionMultiplier,TireConfig.LoadForceRatioAtDoubleLoadLat,LocalState.WheelLoad)*Context.Response.grip_y*Gy;
    Context.PeakForce=FVector2f(Context.AvailableGrip*TireConfig.MaxFx*CachedLUTs.Fx.PeakFriction,
        Context.AvailableGripLat*TireConfig.MaxFy*CachedLUTs.Fy.PeakFriction);
    // Fit peak coefficient independently of source initial slope. No duplicate vehicle
    // friction multiplier: this replaces the imported peak coefficient when declared.
    if(P.reference_mu_x>0) Context.PeakForce.X=Context.AvailableGrip*P.reference_mu_x;
    if(P.reference_mu_y>0) Context.PeakForce.Y=Context.AvailableGripLat*P.reference_mu_y;
    Context.ForceStiffness=FVector2f(
        Grip(TireConfig.FrictionMultiplier,TireConfig.LoadForceRatioAtDoubleLoadLong,LocalState.WheelLoad)*TireConfig.MaxFx*CachedLUTs.Fx.OriginSlope*Context.Response.stiffness_x,
        Grip(TireConfig.FrictionMultiplier,TireConfig.LoadForceRatioAtDoubleLoadLat,LocalState.WheelLoad)*TireConfig.MaxFy*CachedLUTs.Fy.OriginSlope/(PI/2)*Context.Response.stiffness_y);
    const double Cgamma0=Grip(TireConfig.FrictionMultiplier,TireConfig.LoadForceRatioAtDoubleLoadLat,P.reference_load)*TireConfig.MaxFy*CachedLUTs.Fy.OriginSlope/(PI/2)*P.camber_stiffness_ratio;
    LocalState.CamberStiffness=Cgamma0*Context.Response.camber_scale;
    Context.CamberLateralDrift=previs::tire::response::camber_transport(Context.ForceStiffness.Y,
        LocalState.CamberStiffness,Context.Response.gamma,P.transport_limit);
    if(!ContactValid || !Context.Response.valid) {
        Context.PeakForce=FVector2f(0); Context.ForceStiffness=FVector2f(0);
        Context.CamberLateralDrift=0; Context.AvailableGrip=Context.AvailableGripLat=0;
    }
    LocalState.PeakForce=Context.PeakForce; LocalState.ForceStiffness=Context.ForceStiffness;
    LocalState.CamberTransport=Context.CamberLateralDrift;
    LocalState.PeakSlipRatio=Context.ForceStiffness.X>0 ?
        (CachedLUTs.Fx.OptimalSlipIndex/1023.)*CachedLUTs.Fx.OriginSlope/FMath::Max(CachedLUTs.Fx.PeakFriction,SMALL_NUMBER)
        *Context.PeakForce.X/Context.ForceStiffness.X : 0;
    LocalState.PeakSlipAngleRad=Context.ForceStiffness.Y>0 ?
        (CachedLUTs.Fy.OptimalSlipIndex/1023.)*CachedLUTs.Fy.OriginSlope/FMath::Max(CachedLUTs.Fy.PeakFriction,SMALL_NUMBER)
        *Context.PeakForce.Y/Context.ForceStiffness.Y : 0;
    LocalState.EffectivePeakSlipRatio=FMath::Clamp(LocalState.PeakSlipRatio,.005,.99);
    LocalState.SlideAssistance=P.slide_assistance;
    LocalState.ReferenceMuX=P.reference_mu_x; LocalState.ReferenceMuY=P.reference_mu_y;

	// clear tire force
	Context.AccumulateTireImpulse2D = FVector2f(0.f);
}

void FVehicleWheelSolver::Substep(
	float InSubstepDeltaTime,
	float InDriveTorque,
	float InBrakeTorque,
	float InHandbrakeTorque,
	float InReflectedInertia,
	const FVehicleWheelConfig& WheelConfig,
	const FVehicleTireConfig& TireConfig,
	const FVehicleABSConfig& ABSConfig,
	const FVehicleSuspensionSimState& SuspensionState)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(KinetiForgeVehicle_Wheel_WheelSolver_UpdateWheel);

	const FVehicleWheelConfig& Config = WheelConfig;
	FVehicleWheelSimContext& Context = CurrentContext;
	FVehicleWheelSimState& LocalState = State;

	Context.SubstepDeltaTime = InSubstepDeltaTime;
	Context.SubstepDeltaTimeInv = UVehicleUtilities::SafeDivide(1.f, InSubstepDeltaTime);

	LocalState.EffectiveInertia = Config.Inertia + InReflectedInertia;
	LocalState.DriveTorque = InDriveTorque + LocalState.P4MotorTorque;

	float TargetBrakeTorque = FMath::Max(0.f, FMath::Abs(InBrakeTorque) + LocalState.BrakeTorqueFromESP);
	FVehicleABSConfig AdaptiveABS=ABSConfig;
	AdaptiveABS.OptimalSlip=LocalState.EffectivePeakSlipRatio;
	PredictSlipAndUpdateABS(LocalState, Context, AdaptiveABS, TargetBrakeTorque, SuspensionState.bWheelOnGround);

	LocalState.BrakeTorqueFromHandbrake = FMath::Abs(InHandbrakeTorque);
	LocalState.BrakeTorque = LocalState.BrakeTorqueFromBrake + Config.RollingRisistance * Context.Response.rr * (SuspensionState.bWheelOnGround ? 1.f : 0.f) + LocalState.BrakeTorqueFromHandbrake;

    // Apply external drive/brake work before evaluating free contact slip.
    const float PreviousOmega=LocalState.AngularVelocity;
    const float InvI=UVehicleUtilities::SafeDivide(1.f,LocalState.EffectiveInertia);
    LocalState.AngularVelocity += LocalState.DriveTorque*InvI*InSubstepDeltaTime;
    const float UsedBrake=FMath::Min(LocalState.BrakeTorque,FMath::Abs(LocalState.AngularVelocity)*LocalState.EffectiveInertia/ InSubstepDeltaTime);
    LocalState.AngularVelocity -= FMath::Sign(LocalState.AngularVelocity)*UsedBrake*InvI*InSubstepDeltaTime;
    FVector2f SubstepForce2D=SolveTireForce(LocalState,Context, SuspensionState.StaticSprungMass,
        SuspensionState.EffectiveSprungMassLong,SuspensionState.EffectiveSprungMassLat,
        FMath::Max(0.f,SuspensionState.ForceAlongImpactNormal),SuspensionState.bWheelOnGround,TireConfig,CachedLUTs);
    const float SpinSign=FMath::Sign(LocalState.AngularVelocity);
    LocalState.TorqueFromGroundInteraction=-Context.R*(SubstepForce2D.X+SpinSign*Context.CamberLateralDrift*SubstepForce2D.Y);
    LocalState.GroundWorkResidual=SubstepForce2D.X*LocalState.LocalLinearVelocity.X+SubstepForce2D.Y*LocalState.LocalLinearVelocity.Y
        +LocalState.TorqueFromGroundInteraction*LocalState.AngularVelocity+SubstepForce2D.X*LocalState.LongSlipVelocity+SubstepForce2D.Y*LocalState.LatSlipVelocity;
    LocalState.AngularVelocity += LocalState.TorqueFromGroundInteraction*InvI*InSubstepDeltaTime;
    // Remaining brake capacity can hold a wheel which was already stationary.
    const float HoldBrake=FMath::Min(FMath::Max(0.f,LocalState.BrakeTorque-UsedBrake),FMath::Abs(LocalState.AngularVelocity)*LocalState.EffectiveInertia/InSubstepDeltaTime);
    LocalState.AngularVelocity -= FMath::Sign(LocalState.AngularVelocity)*HoldBrake*InvI*InSubstepDeltaTime;
    LocalState.bIsLocked=FMath::IsNearlyZero(LocalState.AngularVelocity)&&LocalState.BrakeTorque>0;
    LocalState.AngularAcceleration=(LocalState.AngularVelocity-PreviousOmega)/InSubstepDeltaTime;
    // Shadow the chassis impulse between substeps; PreStep re-anchors to measured velocity.
    LocalState.LocalLinearVelocity.X += SubstepForce2D.X*InSubstepDeltaTime/FMath::Max(SuspensionState.EffectiveSprungMassLong,1.f);
    LocalState.LocalLinearVelocity.Y += SubstepForce2D.Y*InSubstepDeltaTime/FMath::Max(SuspensionState.EffectiveSprungMassLat,1.f);

	Context.AccumulateTireImpulse2D += SubstepForce2D * Context.SubstepDeltaTime;
}

void FVehicleWheelSolver::PostStep()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(KinetiForgeVehicle_Wheel_WheelSolver_UpdateWheel);

	FVehicleWheelSimContext& Context = CurrentContext;
	FVehicleWheelSimState& LocalState = State;
	LocalState.TireForce2D = Context.AccumulateTireImpulse2D * Context.MacroDeltaTimeInv;
	LocalState.TireForce =
		LocalState.TireForce2D.X * Context.LongForceDir +
		LocalState.TireForce2D.Y * Context.LatForceDir;
}

void FVehicleWheelSolver::DrawWheelForce(
	UVehicleWheelComponent* WheelComponent,
	const FVehicleSuspensionSimState& SuspensionState,
	float Duration, 
	float Thickness, 
	float Length, 
	bool bDrawVelocity, 
	bool bDrawSlip, 
	bool bDrawInertia)
{
	if (!WheelComponent)return;

	UWorld* CurrentWorld = WheelComponent->GetWorld();

	const FVehicleTireConfig& TireConfig = WheelComponent->GetTireConfig();
	FVector WheelRightVec = FVector(SuspensionState.WheelWorldRightVector);
	FVector ImpactNormal = FVector(SuspensionState.ImpactWorldNormal);

	FVector TempForward = FVector::CrossProduct(WheelRightVec, ImpactNormal);
	FVector TempRight = FVector::VectorPlaneProject(WheelRightVec, ImpactNormal);
	FVector TempUp = ImpactNormal;
	FVector TempImpactPoint = SuspensionState.ImpactWorldLocation;
	FRotator TempRot = FRotationMatrix::MakeFromYZ(TempRight, TempForward).Rotator();
    FVector TempScale=FVector(CurrentContext.PeakForce.X,CurrentContext.PeakForce.Y,1);
    FTransform TempTrans=FTransform(TempRot,TempImpactPoint,TempScale);
    Length*=0.01;
    float AvailableGrip=Length;

	//draw grip circle
	FColor GripCircleColor = FColor(0, 191, 255);
	DrawDebugCircle(CurrentWorld, TempTrans.ToMatrixWithScale(), AvailableGrip, 16, GripCircleColor,
		false, Duration, 0, Thickness, false);

	//draw raw force
	FVector Long = TempForward * State.TireForce2D.X * Length;
	FVector Lat = TempRight * State.TireForce2D.Y * Length;
	FColor RawForceColor = FColor(0, 255, 191);
	DrawDebugLine(CurrentWorld, TempImpactPoint, TempImpactPoint + Long, RawForceColor, false, Duration, 0, Thickness);
	DrawDebugLine(CurrentWorld, TempImpactPoint, TempImpactPoint + Lat, RawForceColor, false, Duration, 0, Thickness);

	//draw final force
	DrawDebugLine(CurrentWorld, TempImpactPoint, (FVector)State.TireForce * Length + TempImpactPoint, GripCircleColor, false, Duration, 0, Thickness);

	FTransform HubChassisTransform = FTransform(WheelComponent->GetHubChassisTransform());
	FTransform WheelTrans = HubChassisTransform * WheelComponent->GetChassisAsyncWorldTransform();

	if (bDrawVelocity)
	{
		FColor VelociyColor = FColor(127, 255, 191);
		FColor StringColor = FColor(255 - VelociyColor.R, 255 - VelociyColor.G, 255 - VelociyColor.B);
		FVector TempOffset = WheelRightVec * (SuspensionState.bIsRightWheel ? 1.f : -1.f) * FMath::Abs(State.AngularVelocity) * Length * 100;
		FVector TempWheelLocation = WheelTrans.GetLocation();
		DrawDebugLine(CurrentWorld, TempWheelLocation, TempWheelLocation + TempOffset, VelociyColor, false, Duration, 0, Thickness);

		FString TempString = FString::SanitizeFloat(State.AngularVelocity);
		TempString = FString(TEXT("Omega = ")) + TempString;
		DrawDebugString(CurrentWorld, TempWheelLocation + TempOffset, TempString, 0, VelociyColor, Duration, true);
	}

	if (bDrawSlip)
	{
		FColor SlipColor = FColor(127, 63, 255);
		FVector TempWheelLocation = WheelTrans.GetLocation();
		FVector TempDrawOffset = WheelComponent->GetWheelConfig().Radius * WheelTrans.GetRotation().GetUpVector();
		FString TextSlipRatio = FString(TEXT("SlipRatio = ")) + FString::SanitizeFloat(State.SlipRatio);
		FString TextSlipAngle = FString(TEXT("SlipAngle = ")) + FString::SanitizeFloat(State.SlipAngle);
		DrawDebugString(CurrentWorld, TempWheelLocation + TempDrawOffset, TextSlipRatio, 0, SlipColor, Duration, true, Length * 100);
		DrawDebugString(CurrentWorld, TempWheelLocation - TempDrawOffset, TextSlipAngle, 0, SlipColor, Duration, true, Length * 100);
	}

	if (bDrawInertia)
	{
		FColor InertiaColor = FColor(255, 255, 255);
		FVector TempWheelLocation = WheelTrans.GetLocation();
		FString TextInertia = FString(TEXT("Inertia = ")) + FString::SanitizeFloat(State.EffectiveInertia);
		DrawDebugString(CurrentWorld, TempWheelLocation, TextInertia, 0, InertiaColor, Duration, true, Length * 100);
	}
}

void FVehicleWheelSolver::UpdateCachedLUTs(const FVehicleTireConfig& Config)
{
    // Hash explicit scalar fields, not struct padding; source curve samples join the identity below.
    State.ResponseProfileId=14695981039346656037ULL;
    const auto Hash=[&](double V) {
        uint64 Bits=0; FMemory::Memcpy(&Bits,&V,sizeof(Bits));
        for(int i=0;i<8;++i) { State.ResponseProfileId^=(Bits>>(8*i))&255; State.ResponseProfileId*=1099511628211ULL; }
    };
    const auto& P=Config.ResponseProfile;
    for(double V:{P.reference_pa,P.reference_load,P.camber_reference,P.camber_stiffness_ratio,P.camber_load_power,P.camber_pressure_power,P.transport_limit,P.rr_power,P.flat_grip,P.flat_stiffness,P.flat_rr,P.reference_mu_x,P.reference_mu_y}) Hash(V);
    for(const auto& A:{P.x,P.y}) for(double V:{A.optimum_ratio,A.load_optimum,A.low_width,A.high_width,A.pressure_slope,A.load_slope,A.pressure_quadratic,A.camber_slope,A.pressure_camber,A.camber_optimum}) Hash(V);
	if (IsValid(Config.Fx))
	{
		CachedLUTs.Fx.BuildFromCurve(Config.Fx->FloatCurve);
	}
	else
	{
		CachedLUTs.Fx.SetEstimatedShape(10);
	}
	if (IsValid(Config.Fy))
	{
		CachedLUTs.Fy.BuildFromCurve(Config.Fy->FloatCurve, FVector2f(0.f, 90.f));
	}
	else
	{
		CachedLUTs.Fy.SetEstimatedShape(20);
	}
	//: #494: baked the same way and over the same 0..90 degree domain as the drift curve beside it, so
	//: the two camber curves are read alike. An absent curve stays at 1.0 â€” grip unchanged by camber.
	if (IsValid(Config.CamberToGripFactor))
	{
		CachedLUTs.CamberToGripFactor.CopyFromRichCurve(Config.CamberToGripFactor->FloatCurve, FVector2f(0.f, 90.f));
	}
	else
	{
		CachedLUTs.CamberToGripFactor.SetAllTo(1.f);
	}

    for(int i=0;i<1024;++i) {
        Hash(CachedLUTs.Fx.FastEval(i/1023.f).Value);
        Hash(CachedLUTs.Fy.FastEval(i/1023.f).Value);
        Hash(CachedLUTs.CamberToGripFactor.FastEval(i/1023.f).Value);
    }
    for(double V:{double(Config.MaxFx),double(Config.MaxFy),double(Config.FrictionMultiplier),double(Config.LoadSensitivityReferenceLoad),double(Config.LoadForceRatioAtDoubleLoadLong),double(Config.LoadForceRatioAtDoubleLoadLat),double(Config.RelaxationLength.X),double(Config.RelaxationLength.Y)}) Hash(V);

}

float FVehicleWheelSolver::GetTangentAtOrigin(const FRichCurve& Curve)
{
	if (Curve.Keys.Num() == 0) return 10.f;

	const auto& Key0 = Curve.Keys[0];

	return FMath::IsNearlyZero(Key0.Time) ? Key0.LeaveTangent : 0.f;
}

void FVehicleWheelSolver::PredictSlipAndUpdateABS(
	FVehicleWheelSimState& LocalState,
	const FVehicleWheelSimContext& Context,
	const FVehicleABSConfig& ABSConfig,
	const float TargetBrakeTorque, 
	const bool bOnGround)
{
	// predict angular velocity
	float PredictedOmega = LocalState.AngularVelocity + LocalState.AngularAcceleration * Context.SubstepDeltaTime;
	float PredictedVSlip = PredictedOmega * Context.R - LocalState.LocalLinearVelocity.X;
	PredictedVSlip *= bOnGround;
	float Denominator = FMath::Max(FMath::Max(FMath::Abs(LocalState.LocalLinearVelocity.X), FMath::Abs(PredictedOmega * Context.R)), 1.f);
	LocalState.PredictedSlipRatio = PredictedVSlip / Denominator;

	float AbsolutSlip = FMath::Abs(LocalState.PredictedSlipRatio);
	// only activate abs when the sign of slip and the sign of velocity is different
	bool bDifferentSign = LocalState.PredictedSlipRatio * LocalState.LocalLinearVelocity.X < 0.f;

	LocalState.bABSTriggered = 
		ABSConfig.bAntiBrakeSystemEnabled
		&& TargetBrakeTorque > SMALL_NUMBER
		&& bOnGround
		&& FMath::Abs(LocalState.LocalLinearVelocity.X) > ABSConfig.ActivationSpeed
		&& AbsolutSlip > ABSConfig.OptimalSlip
		&& bDifferentSign;

	if (LocalState.bABSTriggered)
	{
		float Error = AbsolutSlip - ABSConfig.OptimalSlip;
		float AbsFactor = 1.0f - (Error * ABSConfig.Sensitivity);
		AbsFactor = FMath::Clamp(AbsFactor, 0.0f, 1.0f);
		LocalState.BrakeTorqueFromBrake = TargetBrakeTorque * AbsFactor;
		
		return;
	}

	LocalState.BrakeTorqueFromBrake = TargetBrakeTorque;
}

void FVehicleWheelSolver::UpdateDynamicFrictionMultiplier(
	FVehicleWheelSimState& LocalState,
	const FVehicleWheelSimContext& Context,
	const FVehicleTireConfig& TireConfig,
	const float ImpactFriction)
{
	switch (TireConfig.TireFrictionCombineMode)
	{
	case ETireFrictionCombineMode::Constant:
		LocalState.DynFrictionMultiplier = TireConfig.FrictionMultiplier;
		break;
	case ETireFrictionCombineMode::Average:
		LocalState.DynFrictionMultiplier = 0.5 * (TireConfig.FrictionMultiplier + ImpactFriction);
		break;
	case ETireFrictionCombineMode::Multiply:
		LocalState.DynFrictionMultiplier = TireConfig.FrictionMultiplier * ImpactFriction;
		break;
	case ETireFrictionCombineMode::Min:
		LocalState.DynFrictionMultiplier = FMath::Min(TireConfig.FrictionMultiplier, ImpactFriction);
		break;
	case ETireFrictionCombineMode::Max:
		LocalState.DynFrictionMultiplier = FMath::Max(TireConfig.FrictionMultiplier, ImpactFriction);
		break;
	default:
		LocalState.DynFrictionMultiplier = 0.5 * (TireConfig.FrictionMultiplier + ImpactFriction);
		break;
	}
}

void FVehicleWheelSolver::UpdateLinearVelocity(
	FVehicleWheelSimState& LocalState,
	const FVector3f& LongForceDir, 
	const FVector3f& LatForceDir, 
	const FVector3f& ImpactPointWorldVelocity)
{
	LocalState.LocalLinearVelocity.X = FVector3f::DotProduct(LongForceDir, ImpactPointWorldVelocity);
	LocalState.LocalLinearVelocity.Y = FVector3f::DotProduct(LatForceDir, ImpactPointWorldVelocity);
}

void FVehicleWheelSolver::UpdateSlipVelocity(
	FVehicleWheelSimState& LocalState,
	const FVehicleWheelSimContext& Context,
	const bool bOnGround
)
{
	const float Vx = LocalState.LocalLinearVelocity.X;
	const float Vy = LocalState.LocalLinearVelocity.Y;
	const float OmegaR = LocalState.AngularVelocity * Context.R;
	const float AbsOmegaR = FMath::Abs(OmegaR);

	const float q = bOnGround ? Context.CamberLateralDrift : 0.f;


	// Soft patch target. This does not mean rim direction changes.
	const float PatchVx = OmegaR;
	const float PatchVy = AbsOmegaR * q;

	LocalState.LongSlipVelocity = (PatchVx - Vx) * bOnGround;
	LocalState.LatSlipVelocity = (PatchVy - Vy) * bOnGround;

}

void FVehicleWheelSolver::UpdateSlipAngle(
	FVehicleWheelSimState& LocalState,
	const bool bOnGround)
{
	//get velocity2d
	FVector2f Velocity2DNormalized = LocalState.LocalLinearVelocity.GetSafeNormal();

	//calculate lateral slip
	//get slip angle
	float SlipAngleRaw = FMath::Asin(-Velocity2DNormalized.Y);
	SlipAngleRaw = FMath::RadiansToDegrees(SlipAngleRaw);

	//approximate low speed slip angle
	//-V.y = |V| * sin(SlipAngle) ¡Ö |V| * SlipAngle ¡Ö SlipAngle; when slip angle is small and |v| --> 1.f
	float LowSpeedSlipAngle = -FMath::RadiansToDegrees(LocalState.LocalLinearVelocity.Y);

	//get alpha for lerp, lerp between high speed and low speede
	float Alpha = FMath::GetMappedRangeValueClamped(FVector2f(0.01f, 0.1f), FVector2f(0.f, 1.f), LocalState.LocalLinearVelocity.SquaredLength());

	//combine low speed and high speed
	LocalState.SlipAngle = FMath::Lerp(LowSpeedSlipAngle, SlipAngleRaw, Alpha) * bOnGround;
	LocalState.SlipAngle = FMath::Clamp(LocalState.SlipAngle, -90.f, 90.f);
}

void FVehicleWheelSolver::UpdateSlipRatio(
	FVehicleWheelSimState& LocalState,
	const FVehicleWheelSimContext& Context,
	const bool bOnGround)
{
	float KinematicsLongSlipVelocity = LocalState.AngularVelocity * Context.R - LocalState.LocalLinearVelocity.X;
	KinematicsLongSlipVelocity *= bOnGround;
	float Denominator = FMath::Max(FMath::Max(FMath::Abs(LocalState.LocalLinearVelocity.X), FMath::Abs(LocalState.AngularVelocity * Context.R)), 1.f);
	LocalState.SlipRatio = KinematicsLongSlipVelocity / Denominator;
}

FVector2f FVehicleWheelSolver::UpdateTransientSlip(
	FVehicleWheelSimState& LocalState,
	const FVehicleWheelSimContext& Context,
	const bool bOnGround, 
	const FVector2f& RelaxationLength)
{
	if (bOnGround)
	{
		const FVector2f SafeRelaxationLength = FVector2f::Max(RelaxationLength, FVector2f(SMALL_NUMBER));
		const FVector2f RelaxationLengthInv = FVector2f(1.f) / SafeRelaxationLength;

		const FVector2f SlipVelocity =
			FVector2f(LocalState.LongSlipVelocity, LocalState.LatSlipVelocity);

		// This is only for the relaxation denominator.
		// It is a numerical regularization, not a real transport speed.
		float AbsVx = FMath::Abs(LocalState.LocalLinearVelocity.X);
		float AbsOmegaR = FMath::Abs(LocalState.AngularVelocity * Context.R);
		FVector2f AbsVx2D = FVector2f(FMath::Max(AbsVx, AbsOmegaR), AbsVx);

		float MinVx = 0.1f;
		AbsVx2D = FVector2f::Max(AbsVx2D, FVector2f(MinVx));

		LocalState.TransientSlip =
			(LocalState.TransientSlip + (SlipVelocity * Context.SubstepDeltaTime) * RelaxationLengthInv) /
			(FVector2f(1.f, 1.f) + (AbsVx2D * Context.SubstepDeltaTime) * RelaxationLengthInv);

		float TransientSlipRatio = LocalState.TransientSlip.X;
		float TransientSlipAngle = FMath::Atan(LocalState.TransientSlip.Y) / (0.5f * PI);

		return FVector2f(TransientSlipRatio, TransientSlipAngle);
	}
	else
	{
		LocalState.TransientSlip = FVector2f(0.f, 0.f);
		return FVector2f(0.f, 0.f);
	}
}

FVector2f FVehicleWheelSolver::CalculateGravityCompensationOnSlope(
	FVehicleWheelSimState& LocalState,
	FVehicleWheelSimContext& Context,
	const float PositiveForceIntoSurface,
	const bool bOnGround,
	const FVector3f& LongForceDir, 
	const FVector3f& LatForceDir)
{
	if (!bOnGround)
	{
		return FVector2f(0.f);
	}

	FVector2f GravityComp = FVector2f(0.f, 0.f);

	// sin(theta): LatForceDir.Z or LongForceDir.Z
	// ExtraGravityForce = Fz * TanTheta; 
	// TanTheta ¡Ö SinTheta, when theta is small; 
	// when theta is large, the vehicle maybe should not be able to hold itself on the slope; 
	// so there might be no need to calculate tan(theta)
	   
	//Lateral:
	float SlipSign = FMath::Sign(LocalState.LatSlipVelocity);
	GravityComp.Y = LatForceDir.Z * PositiveForceIntoSurface;

	// longitudinal:
	if (LocalState.bIsLocked)
	{
		SlipSign = FMath::Sign(LocalState.LongSlipVelocity);
		GravityComp.X = LongForceDir.Z * PositiveForceIntoSurface;

		//consider brake force?
		float BrakeForce = LocalState.BrakeTorque * Context.RInv;	//LocalState.BrakeTorque is always positive
		GravityComp.X = FMath::Clamp(GravityComp.X, -BrakeForce, BrakeForce);
	}

	// in the lateral direction, the wheel can be treated as it is always braking

	Context.GravityComp2D = GravityComp;
	return GravityComp;
}

float FVehicleWheelSolver::CalculateAvailableGrip(
	const float FrictionMultiplier,
	const float StaticSprungMass,
	const float WheelLoad,
	const float Saturation,
	const float ReferenceLoad,
	const float DoubleLoadForceRatio)
{
	const float DefaultGravity = 9.81f;
	//: ADR-031: PROJECT CHRONO'S OWN LOAD LAW (ChTMeasyTire::InterpQ), which is the shape this
	//: extension carries and every source is mapped into. Quadratic through the origin, through the
	//: peak force at the reference load, and through the peak force at twice it; clamped at 3.5x the
	//: reference load, which is Chrono's pn_max. The curve carries the coefficient, so this is the
	//: load term alone.
	if (ReferenceLoad > 0.f && DoubleLoadForceRatio > 0.f)
	{
		const float q = FMath::Clamp(FMath::Max(WheelLoad, 0.f) / ReferenceLoad, 0.f, 3.5f);
		const float HalfRatio = 0.5f * DoubleLoadForceRatio;
		const float Scale = q * (2.f - HalfRatio - (1.f - HalfRatio) * q);
		return FrictionMultiplier * ReferenceLoad * FMath::Max(Scale, 0.f);
	}
	float NormWheelLoad = StaticSprungMass * DefaultGravity;
	float LoadRatio = UVehicleUtilities::SafeDivide(WheelLoad, NormWheelLoad);
	float b = (1.f - Saturation) / (2.f + 2.f * Saturation);
	float LoadScale = LoadRatio / (1.f + b * LoadRatio);
	return FrictionMultiplier * LoadScale * NormWheelLoad;
}

FVector2f FVehicleWheelSolver::SolveTireForce(
    FVehicleWheelSimState& LocalState,const FVehicleWheelSimContext& Context,
    const float StaticSprungMass,const float EffectiveSprungMassLong,const float EffectiveSprungMassLat,
    const float PositiveForceIntoSurface,const bool bOnGround,const FVehicleTireConfig& TireConfig,
    const FVehicleWheelCachedLUTs& TireLUTs)
{
    namespace tr=previs::tire::response;
    const bool Active=bOnGround && PositiveForceIntoSurface>0 && Context.Response.valid;
    UpdateSlipVelocity(LocalState,Context,Active);
    const FVector2f Slip=UpdateTransientSlip(LocalState,Context,Active,TireConfig.RelaxationLength);
    UpdateSlipRatio(LocalState,Context,Active); UpdateSlipAngle(LocalState,Active);
    LocalState.TargetForce=FVector2f(0); LocalState.ImpulseScale=1; LocalState.LocalImpulseEnergy=0;
    if(!Active) return FVector2f(0);
    const double Dx=Context.PeakForce.X,Dy=Context.PeakForce.Y;
    const auto X=[&](double r) { const double base=double(TireLUTs.Fx.FastEval(r*TireLUTs.Fx.PeakFriction/FMath::Max(TireLUTs.Fx.OriginSlope,SMALL_NUMBER)).Value)/FMath::Max(TireLUTs.Fx.PeakFriction,SMALL_NUMBER);
        const double peak=(TireLUTs.Fx.OptimalSlipIndex/1023.)*TireLUTs.Fx.OriginSlope/FMath::Max(TireLUTs.Fx.PeakFriction,SMALL_NUMBER);
        return tr::slide_tail(base,r,peak,TireConfig.ResponseProfile.slide_assistance); };
    const auto Y=[&](double r) { const double base=double(TireLUTs.Fy.FastEval(r*TireLUTs.Fy.PeakFriction/FMath::Max(TireLUTs.Fy.OriginSlope,SMALL_NUMBER)).Value)/FMath::Max(TireLUTs.Fy.PeakFriction,SMALL_NUMBER);
        const double peak=(TireLUTs.Fy.OptimalSlipIndex/1023.)*TireLUTs.Fy.OriginSlope/FMath::Max(TireLUTs.Fy.PeakFriction,SMALL_NUMBER);
        return tr::slide_tail(base,r,peak,TireConfig.ResponseProfile.slide_assistance); };
    const auto F=tr::force(Dx,Dy,Context.ForceStiffness.X,Context.ForceStiffness.Y,{Slip.X,Slip.Y*(PI/2)},X,Y);
    LocalState.TargetForce=FVector2f(F.x,F.y);
    const double dt=Context.SubstepDeltaTime, mx=FMath::Max(EffectiveSprungMassLong,1.f),my=FMath::Max(EffectiveSprungMassLat,1.f);
    const double jx=Context.R,jy=Context.R*FMath::Sign(LocalState.AngularVelocity)*Context.CamberLateralDrift;
    const double invI=1/FMath::Max(LocalState.EffectiveInertia,SMALL_NUMBER);
    const double axx=1/mx+jx*jx*invI,axy=jx*jy*invI,ayy=1/my+jy*jy*invI;
    tr::Vec u{LocalState.LongSlipVelocity,LocalState.LatSlipVelocity};
    double blend=FMath::Clamp((FMath::Max(FMath::Abs(LocalState.LocalLinearVelocity.X),FMath::Abs(LocalState.AngularVelocity*Context.R))-.5)/1.5,0.,1.);
    blend=blend*blend*(3-2*blend); LocalState.LowSpeedBlend=blend;
    // External slope acceleration enters only the low-speed static predictor.
    u.x+=(1-blend)*dt*Context.GravityComp2D.X/mx;
    u.y+=(1-blend)*dt*Context.GravityComp2D.Y/my;
    const double det=axx*ayy-axy*axy;
    tr::Vec j{blend*dt*F.x+(1-blend)*(ayy*u.x-axy*u.y)/det,
              blend*dt*F.y+(1-blend)*(axx*u.y-axy*u.x)/det};
    if(Dx<=0) j.x=0; if(Dy<=0) j.y=0;
    const double envelope=std::hypot(Dx>0?j.x/(dt*Dx):0,Dy>0?j.y/(dt*Dy):0);
    if(envelope>1) { j.x/=envelope; j.y/=envelope; }
    double scale=tr::passive_scale(u,j,axx,axy,ayy);
    const double spinDelta=-(jx*j.x+jy*j.y)*invI;
    if(Context.CamberLateralDrift!=0 && LocalState.AngularVelocity*spinDelta<0)
        scale=std::min(scale,std::abs(LocalState.AngularVelocity/spinDelta));
    LocalState.ImpulseScale=scale;
    j.x*=scale; j.y*=scale;
    LocalState.LocalImpulseEnergy=-u.x*j.x-u.y*j.y+.5*(axx*j.x*j.x+2*axy*j.x*j.y+ayy*j.y*j.y);
    return FVector2f(j.x/dt,j.y/dt);
}
