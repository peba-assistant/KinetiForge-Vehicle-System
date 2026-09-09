// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;
using System.Security.Cryptography;
using System.Text;

public class KinetiForge : ModuleRules
{
	public KinetiForge(ReadOnlyTargetRules Target) : base(Target)
	{
		PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../../Core/include")));
        // Exact model source identity, including uncommitted fork builds.
        using (var sha = SHA256.Create()) {
            string body = File.ReadAllText(Path.Combine(ModuleDirectory,"Private/VehicleWheelSolver.cpp"), Encoding.Latin1)
                + File.ReadAllText(Path.Combine(ModuleDirectory,"Public/VehicleWheelStructs.h"), Encoding.Latin1)
                + File.ReadAllText(Path.Combine(ModuleDirectory,"../../../../../Core/include/previs/tire_response.hpp"));
            string id = System.BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(body))).Replace("-", "").ToLowerInvariant();
            PublicDefinitions.Add("KINETIFORGE_TYRE_BUILD_ID=\"" + id + "\"");
        }
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
				"Chaos",
                "ChaosSolverEngine",
                "PhysicsCore",
                "AsyncTickPhysics"
            }
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				// ... add private dependencies that you statically link with here ...	
				"InputCore", 
				"Chaos", 
				"ChaosSolverEngine", 
				"PhysicsCore",
                "AsyncTickPhysics",
                "MovieScene",
                "NavigationSystem"
            }
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
