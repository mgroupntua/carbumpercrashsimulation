using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Line;
using MGroup.FEM.Structural.Continuum;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.MSolve.Discretization;
using System.Collections.Generic;
using System.Linq;
using System;
//using MGroup.Contact.Structural;

namespace MGroup.CarBumperCrashSimulation
{
	public class TruckBumperImpact
	{
        public static readonly double expected_solution_node862_TranslationZ = 0.025324;//Velocity 5d, 0.0475sec, peek

        //public static readonly double expected_solution_node1105_TranslationZ = 1.4061e-002;//Velocity 5d, 0.0475sec, VonMisesMaterial3D 15%
        //public static readonly double expected_solution_node1105_TranslationZ = 0.085521;//Velocity 25d, 0.0055sec, ElasticMaterial3D
        //public static readonly double expected_solution_node1105_TranslationZ = 0.13072;//Velocity 15d, 0.015sec, VonMisesMaterial3D
        //public static readonly double expected_solution_node1105_TranslationZ = 0.0733;//Velocity 10d, 0.0127sec, VonMisesMaterial3D
        //const double yStress = 275000000d;
        //const double yStress = 475000000d;
        const double yStress = 550000000d;
        //const double hardening = 85714285714.29;
        //const double hardening = 11394144382.2006;
        //const double hardening = 35294117647.0588;
        const double hardening = 53750000000.00;

        public static Model CreateModel(ImportedData importedData, double modulusOfElasticity)
		{
			var model = new Model();
            var YoungMod = modulusOfElasticity;
            var poissonRatio = 0.40;
            //var density = 7850d;
            var density = 1320d;

            var epsilonP = 2d;
            //var epsilonP = 5d;
            var contactNoIntegrationPoints = 3;
			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = importedData.nodes;

            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.Key, node.Value);
            }
			var slaveConnect = importedData.slaveConnectivity;
			var masterConnect = importedData.masterConnectivity;
			var contElementsConnect = importedData.elementsConnectivity;
            var fixedNodes = importedData.fixedNodes;
			var elementId = 1;
			for(var i = 1; i <= 1760; i++)
            {
                var element = new SolidShellEAS7ANS
                    (
                    nodes: new[]
                    {
                    model.NodesDictionary[contElementsConnect[i][1]],
                    model.NodesDictionary[contElementsConnect[i][2]],
                    model.NodesDictionary[contElementsConnect[i][3]],
                    model.NodesDictionary[contElementsConnect[i][4]],
                    model.NodesDictionary[contElementsConnect[i][5]],
                    model.NodesDictionary[contElementsConnect[i][6]],
                    model.NodesDictionary[contElementsConnect[i][7]],
                    model.NodesDictionary[contElementsConnect[i][8]]
                    },
                //new ElasticMaterial3D(youngModulus: YoungMod, poissonRatio: poissonRatio),
                //new VonMisesMaterial3D(youngModulus: YoungMod, poissonRatio: poissonRatio, yieldStress: 0.45 * 1e9, hardeningRatio: 53.75 * 1e9),
                new VonMisesMaterial3D(youngModulus: YoungMod, poissonRatio: poissonRatio, yieldStress: 0.025 * 1e9, hardeningRatio: 1.4 * 1e9),
                //new DruckerPrager3DFunctional(YoungMod, poissonRatio, 0, 0, 0.05 * 1e9, x => (0.05 * 1e9 + (0.75 * 1e9 * x))),
                //new DruckerPrager3DFunctional(youngModulus: 3.5 * 1e9, poissonRatio: 0.4, friction: 20, dilation: 20, cohesion: 0.025 * 1e9, hardeningFunc: x => (0.025 * 1e9 + 0.35 * 1e9 * x)),
                dynamicProperties: new TransientAnalysisProperties(density, 0, 0)
					);
                element.ID = elementId;
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
                elementId += 1;
            }
			var Continuum3dElementFactory = new ContinuumElement3DFactory(
            //new ElasticMaterial3D(youngModulus: 1000 * 1e9, poissonRatio: 0.20),
            //commonDynamicProperties: new TransientAnalysisProperties(3150d, 0, 0)
            new ElasticMaterial3D(youngModulus: 1.8 * 1e9, poissonRatio: 0.20),
            commonDynamicProperties: new TransientAnalysisProperties(160d, 0, 0)
            );
            //-----------------------------------------------------------------------------------------------------------------------
            //Old impactor model
            //-----------------------------------------------------------------------------------------------------------------------
            for (var i = 1761; i <= 2820; i++)
            {
                var element = Continuum3dElementFactory.CreateElement(
                    CellType.Hexa8,
                    nodes: new[]
                    {
                    model.NodesDictionary[contElementsConnect[i][7]],
                    model.NodesDictionary[contElementsConnect[i][8]],
                    model.NodesDictionary[contElementsConnect[i][5]],
                    model.NodesDictionary[contElementsConnect[i][6]],
                    model.NodesDictionary[contElementsConnect[i][3]],
                    model.NodesDictionary[contElementsConnect[i][4]],
                    model.NodesDictionary[contElementsConnect[i][1]],
                    model.NodesDictionary[contElementsConnect[i][2]]
                    }
                    );
                element.ID = elementId;
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
                elementId += 1;
            }
            //-----------------------------------------------------------------------------------------------------------------------
            //for (var i = 1761; i <= 1976; i++)
            //{
            //    var element = Continuum3dElementFactory.CreateElement(
            //        CellType.Hexa8,
            //        nodes: new[]
            //        {
            //        model.NodesDictionary[contElementsConnect[i][6]],
            //        model.NodesDictionary[contElementsConnect[i][7]],
            //        model.NodesDictionary[contElementsConnect[i][8]],
            //        model.NodesDictionary[contElementsConnect[i][5]],
            //        model.NodesDictionary[contElementsConnect[i][2]],
            //        model.NodesDictionary[contElementsConnect[i][3]],
            //        model.NodesDictionary[contElementsConnect[i][4]],
            //        model.NodesDictionary[contElementsConnect[i][1]]
            //        }
            //        );
            //    element.ID = elementId;
            //    model.ElementsDictionary.Add(element.ID, element);
            //    model.SubdomainsDictionary[0].Elements.Add(element);
            //    elementId += 1;
            //}
            int connectCount = model.ElementsDictionary.Count + 1;
            //-----------------------------------------------------------------------------------------------------------------------
            //Old impactor model
            //-----------------------------------------------------------------------------------------------------------------------
            foreach (var nodeList in masterConnect)
            {
                if (nodeList.Key <= 8)
                {
                    for (int i = 65; i <= 80; i++)
                    {
                        var contactElement = new ContactSurfaceToSurface3D(
                        new[]
                        {
                            model.NodesDictionary[slaveConnect[i][1]],
                            model.NodesDictionary[slaveConnect[i][2]],
                            model.NodesDictionary[slaveConnect[i][3]],
                            model.NodesDictionary[slaveConnect[i][4]],
                            model.NodesDictionary[nodeList.Value[1]],
                            model.NodesDictionary[nodeList.Value[2]],
                            model.NodesDictionary[nodeList.Value[3]],
                            model.NodesDictionary[nodeList.Value[4]]
                        },
                        youngModulus: YoungMod,
                        penaltyFactorMultiplier: epsilonP,
                        contactArea: 1d,
                        masterSurfaceOrder: 1,
                        slaveSurfaceOrder: 1,
                        integrationPointsPerNaturalAxis: contactNoIntegrationPoints
                        );
                        contactElement.ID = connectCount;
                        model.ElementsDictionary.Add(contactElement.ID, contactElement);
                        model.SubdomainsDictionary[0].Elements.Add(contactElement);
                        connectCount += 1;
                    }
                }
                if ((nodeList.Key >= 9 && nodeList.Key <= 32) ||
                    (nodeList.Key >= 65 && nodeList.Key <= 72) ||
                   (nodeList.Key >= 97 && nodeList.Key <= 104))
                {
                    for (int i = 57; i <= 80; i++)
                    {
                        var contactElement = new ContactSurfaceToSurface3D(
                        new[]
                        {
                            model.NodesDictionary[slaveConnect[i][1]],
                            model.NodesDictionary[slaveConnect[i][2]],
                            model.NodesDictionary[slaveConnect[i][3]],
                            model.NodesDictionary[slaveConnect[i][4]],
                            model.NodesDictionary[nodeList.Value[1]],
                            model.NodesDictionary[nodeList.Value[2]],
                            model.NodesDictionary[nodeList.Value[3]],
                            model.NodesDictionary[nodeList.Value[4]]
                        },
                        youngModulus: YoungMod,
                        penaltyFactorMultiplier: epsilonP,
                        contactArea: 1d,
                        masterSurfaceOrder: 1,
                        slaveSurfaceOrder: 1,
                        integrationPointsPerNaturalAxis: contactNoIntegrationPoints
                        );
                        contactElement.ID = connectCount;
                        model.ElementsDictionary.Add(contactElement.ID, contactElement);
                        model.SubdomainsDictionary[0].Elements.Add(contactElement);
                        connectCount += 1;
                    }
                }
                if (nodeList.Key >= 65 && nodeList.Key <= 112)
                {
                    for (int i = 25; i <= 56; i++)
                    {
                        var contactElement = new ContactSurfaceToSurface3D(
                        new[]
                        {
                            model.NodesDictionary[slaveConnect[i][1]],
                            model.NodesDictionary[slaveConnect[i][2]],
                            model.NodesDictionary[slaveConnect[i][3]],
                            model.NodesDictionary[slaveConnect[i][4]],
                            model.NodesDictionary[nodeList.Value[1]],
                            model.NodesDictionary[nodeList.Value[2]],
                            model.NodesDictionary[nodeList.Value[3]],
                            model.NodesDictionary[nodeList.Value[4]]
                        },
                        youngModulus: YoungMod,
                        penaltyFactorMultiplier: epsilonP,
                        contactArea: 1d,
                        masterSurfaceOrder: 1,
                        slaveSurfaceOrder: 1,
                        integrationPointsPerNaturalAxis: contactNoIntegrationPoints
                        );
                        contactElement.ID = connectCount;
                        model.ElementsDictionary.Add(contactElement.ID, contactElement);
                        model.SubdomainsDictionary[0].Elements.Add(contactElement);
                        connectCount += 1;
                    }
                }
                if ((nodeList.Key >= 33 && nodeList.Key <= 40))
                {
                    for (int i = 1; i <= 16; i++)
                    {
                        var contactElement = new ContactSurfaceToSurface3D(
                        new[]
                        {
                            model.NodesDictionary[slaveConnect[i][1]],
                            model.NodesDictionary[slaveConnect[i][2]],
                            model.NodesDictionary[slaveConnect[i][3]],
                            model.NodesDictionary[slaveConnect[i][4]],
                            model.NodesDictionary[nodeList.Value[1]],
                            model.NodesDictionary[nodeList.Value[2]],
                            model.NodesDictionary[nodeList.Value[3]],
                            model.NodesDictionary[nodeList.Value[4]]
                        },
                        youngModulus: YoungMod,
                        penaltyFactorMultiplier: epsilonP,
                        contactArea: 1d,
                        masterSurfaceOrder: 1,
                        slaveSurfaceOrder: 1,
                        integrationPointsPerNaturalAxis: contactNoIntegrationPoints
                        );
                        contactElement.ID = connectCount;
                        model.ElementsDictionary.Add(contactElement.ID, contactElement);
                        model.SubdomainsDictionary[0].Elements.Add(contactElement);
                        connectCount += 1;
                    }
                }
                if ((nodeList.Key >= 41 && nodeList.Key <= 64) ||
                   (nodeList.Key >= 73 && nodeList.Key <= 80) ||
                   (nodeList.Key >= 105 && nodeList.Key <= 112))
                {
                    for (int i = 1; i <= 24; i++)
                    {

                        var contactElement = new ContactSurfaceToSurface3D(
                        new[]
                        {
                            model.NodesDictionary[slaveConnect[i][1]],
                            model.NodesDictionary[slaveConnect[i][2]],
                            model.NodesDictionary[slaveConnect[i][3]],
                            model.NodesDictionary[slaveConnect[i][4]],
                            model.NodesDictionary[nodeList.Value[1]],
                            model.NodesDictionary[nodeList.Value[2]],
                            model.NodesDictionary[nodeList.Value[3]],
                            model.NodesDictionary[nodeList.Value[4]]
                        },
                        youngModulus: YoungMod,
                        penaltyFactorMultiplier: epsilonP,
                        contactArea: 1d,
                        masterSurfaceOrder: 1,
                        slaveSurfaceOrder: 1,
                        integrationPointsPerNaturalAxis: contactNoIntegrationPoints
                        );
                        contactElement.ID = connectCount;
                        model.ElementsDictionary.Add(contactElement.ID, contactElement);
                        model.SubdomainsDictionary[0].Elements.Add(contactElement);
                        connectCount += 1;
                    }
                }
            }
            //-----------------------------------------------------------------------------------------------------------------------
            //foreach (var nodeList in masterConnect)
            //{
            //    for (int i = 1; i <= 84; i++)
            //    {
            //        var contactElement = new ContactSurfaceToSurface3D(
            //            new[]
            //            {
            //                model.NodesDictionary[slaveConnect[i][1]],
            //                model.NodesDictionary[slaveConnect[i][2]],
            //                model.NodesDictionary[slaveConnect[i][3]],
            //                model.NodesDictionary[slaveConnect[i][4]],
            //                model.NodesDictionary[nodeList.Value[1]],
            //                model.NodesDictionary[nodeList.Value[2]],
            //                model.NodesDictionary[nodeList.Value[3]],
            //                model.NodesDictionary[nodeList.Value[4]]
            //            },
            //            youngModulus: YoungMod,
            //            penaltyFactorMultiplier: epsilonP,
            //            contactArea: 1d,
            //            masterSurfaceOrder: 1,
            //            slaveSurfaceOrder: 1,
            //            integrationPointsPerNaturalAxis: contactNoIntegrationPoints
            //            );
            //        contactElement.ID = connectCount;
            //        model.ElementsDictionary.Add(contactElement.ID, contactElement);
            //        model.SubdomainsDictionary[0].Elements.Add(contactElement);
            //        connectCount += 1;
            //    }
            //}
            //-----------------------------------------------------------------------------------------------------------------------
            var constraints = new List<INodalDisplacementBoundaryCondition>();
            foreach(var node in fixedNodes)
            {
                constraints.Add(new NodalDisplacement(model.NodesDictionary[node], StructuralDof.TranslationX, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[node], StructuralDof.TranslationY, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[node], StructuralDof.TranslationZ, amount: 0d));
            }
            constraints = constraints.Distinct().ToList();
            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
                constraints,
                new NodalLoad[] { }
            ));
            var velocities = new List<INodalVelocityBoundaryCondition>();
            //-----------------------------------------------------------------------------------------------------------------------
            //Old impactor model
            //-----------------------------------------------------------------------------------------------------------------------
            for (var n = 3727; n <= 5134; n++)
            {
                velocities.Add(new NodalVelocity(model.NodesDictionary[n], StructuralDof.TranslationZ, amount: 2.7778));
            }
            //-----------------------------------------------------------------------------------------------------------------------
            //for (var n = 3727; n <= 4069; n++)
            //{
            //    velocities.Add(new NodalVelocity(model.NodesDictionary[n], StructuralDof.TranslationZ, amount: 5d));
            //}
            //-----------------------------------------------------------------------------------------------------------------------
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet
            (
               new[] { new StructuralBoundaryConditionSet(
                     velocities,
                     null
                 )},
               (t, amount) => t == 0d ? amount : 0d
            ));
            return model;
		}
	}
}
