using BenchmarkDotNet.Running;
using SoftBodyBenchmarks;

if (args.Length == 1 && args[0] == "--verify")
{
    return ClothRigidBodyBenchmarks.VerifyZeroSteadyStateAllocations();
}

BenchmarkSwitcher
    .FromAssembly(typeof(ClothRigidBodyBenchmarks).Assembly)
    .Run(args.Length > 0 ? args : ["--filter", "*"]);
return 0;
