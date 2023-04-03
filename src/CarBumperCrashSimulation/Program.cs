using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;

namespace MGroup.CarBumperCrashSimulation
{
    static class Program
    {
        private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();
        
        private static DOFSLog SolveModel(Model model)
        {
            var solverFactory = new SkylineSolver.Factory();
            //var solverFactory = new SuiteSparseSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel);

            //var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 1);
            loadControlAnalyzerBuilder.MaxIterationsPerIncrement = 100;
            loadControlAnalyzerBuilder.ResidualTolerance = 0.0001;
            //loadControlAnalyzerBuilder.NumIterationsForMatrixRebuild = 100;
            var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
            //var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, problem, loadControlAnalyzer, timeStep: 0.000475, totalTime: 0.00475);
            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(algebraicModel, problem, loadControlAnalyzer, timeStep: 0.0002, totalTime: 0.015);
            //var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(algebraicModel, problem, loadControlAnalyzer, timeStep: 0.0004, totalTime: 0.1);

            //dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.3025, gamma: 0.6, allowConditionallyStable: false);
            dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
            //dynamicAnalyzerBuilder.SetNewmarkParametersForLinearAcceleration();

            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            //watchDofs.Add((model.NodesDictionary[1105], StructuralDof.TranslationZ));
            watchDofs.Add((model.NodesDictionary[862], StructuralDof.TranslationZ));
            loadControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            //return (DOFSLog)linearAnalyzer.Logs[0];
            return (DOFSLog)loadControlAnalyzer.Logs[0];
        }

        static void Main(string[] args)
        {
            ImportedData importedData = new ImportedData();
            importedData.ImportSlaveSurfaceConnectivity(@"..\..\..\Data\SlaveSurfaceConnectivity.txt");
            importedData.ImportMasterSurfaceConnectivity(@"..\..\..\Data\MasterSurfaceConnectivity.txt");
            importedData.ImportNodes(@"..\..\..\Data\BumperImpactNodes.txt");
            //importedData.ImportFixedNodes("BumperImpactFixedNodesList.txt");
            importedData.ImportElementsConnectivity(@"..\..\..\Data\BumperImpactConnectivity.txt");
            //importedData.ImportSlaveSurfaceConnectivity("TruckImpactor2SlaveConnect.txt");
            //importedData.ImportMasterSurfaceConnectivity("TruckImpactor2MasterConnect.txt");
            //importedData.ImportNodes("TruckImpactor2Nodes.txt");
            importedData.ImportFixedNodes(@"..\..\..\Data\TruckImpactor2ConstrainedNodes.txt");
            //importedData.ImportElementsConnectivity("TruckImpactor2Connectivity.txt");
            //var E = 215.0 * 1e9;
            var E = 3.5 * 1e9;
            var model = TruckBumperImpact.CreateModel(importedData, E);
            var log = SolveModel(model);
        }
    }
}
