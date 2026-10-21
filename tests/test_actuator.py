# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

import math
import unittest

from pxr import Plug, Sdf, Usd, UsdGeom

import newton_usd_schemas  # noqa: F401

USD_HAS_LIMITS = Usd.GetVersion() >= (0, 25, 11)


class TestNewtonActuator(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_typed_schema_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuator")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuator")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuator")
        self.assertEqual(schema_type, "NewtonActuator")

    def test_prim_is_actuator(self):
        self.assertEqual(self.prim.GetTypeName(), "NewtonActuator")
        self.assertTrue(self.prim.IsValid())
        self.assertTrue(self.prim.HasRelationship("newton:targets"))

    def test_targets_relationship(self):
        rel = self.prim.GetRelationship("newton:targets")
        self.assertFalse(rel.HasAuthoredTargets())
        rel.SetTargets([Sdf.Path("/World/Joint")])
        targets = rel.GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(str(targets[0]), "/World/Joint")

    def test_not_imageable(self):
        # Actuators carry a control law bound to a joint; they have no extent and are
        # never rendered, so they derive from UsdTyped rather than UsdGeomImageable.
        self.assertFalse(self.prim.IsA(UsdGeom.Imageable))

        # The prim definition must not advertise Imageable properties it does not have.
        definition = Usd.SchemaRegistry().FindConcretePrimDefinition("NewtonActuator")
        self.assertIsNotNone(definition)
        self.assertEqual(sorted(definition.GetPropertyNames()), ["newton:targets"])

    def test_bbox_cache_skips_actuator(self):
        # Regression test: when NewtonActuator declared UsdGeomImageable as its base
        # without baking in the inherited visibility/purpose/proxyPrim properties,
        # UsdGeom.BBoxCache treated the prim as Imageable and then failed to read its
        # visibility attribute, raising from _GetOrCreateVisibilityQuery.
        root = self.stage.DefinePrim("/World", "Xform")
        self.stage.DefinePrim("/World/Actuator", "NewtonActuator")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bound = cache.ComputeWorldBound(root)
        self.assertTrue(bound.GetRange().IsEmpty())


class TestNewtonBattery(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Battery", "NewtonBattery")

    def test_typed_schema_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsBattery")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsBattery")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsBattery")
        self.assertEqual(schema_type, "NewtonBattery")

    def test_prim_is_battery(self):
        self.assertEqual(self.prim.GetTypeName(), "NewtonBattery")
        self.assertTrue(self.prim.IsValid())
        self.assertFalse(self.prim.IsA(UsdGeom.Imageable))
        self.assertTrue(self.prim.HasRelationship("newton:joints"))
        for name in ("nominalVoltage", "sagGain", "minVoltage"):
            self.assertTrue(self.prim.HasAttribute(f"newton:{name}"), name)

    def test_joints_relationship(self):
        rel = self.prim.GetRelationship("newton:joints")
        self.assertFalse(rel.HasAuthoredTargets())
        rel.SetTargets([Sdf.Path("/World/Knee"), Sdf.Path("/World/Ankle")])
        self.assertEqual([str(t) for t in rel.GetTargets()], ["/World/Knee", "/World/Ankle"])

    def test_voltages(self):
        for name, fallback, value in (("nominalVoltage", None, 12.0), ("sagGain", 0.0, 0.5), ("minVoltage", None, 9.0)):
            with self.subTest(name=name):
                attr = self.prim.GetAttribute(f"newton:{name}")
                self.assertFalse(attr.HasAuthoredValue())
                if fallback is None:
                    self.assertIsNone(attr.Get())
                else:
                    self.assertAlmostEqual(attr.Get(), fallback)

                attr.Set(value)
                self.assertTrue(attr.HasAuthoredValue())
                self.assertAlmostEqual(attr.Get(), value)

                if USD_HAS_LIMITS:
                    hard = attr.GetHardLimits()
                    self.assertTrue(hard.IsValid())
                    self.assertAlmostEqual(hard.GetMinimum(), 0.0)
                    self.assertIsNone(hard.GetMaximum())


class TestNewtonActuatorInputProcessorBaseAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuatorInputProcessorBaseAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuatorInputProcessorBaseAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuatorInputProcessorBaseAPI")
        self.assertEqual(schema_type, "NewtonActuatorInputProcessorBaseAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonActuatorInputProcessorBaseAPI"))
        self.prim.ApplyAPI("NewtonActuatorInputProcessorBaseAPI")
        self.assertTrue(self.prim.HasAPI("NewtonActuatorInputProcessorBaseAPI"))

    def test_applied_by_input_processors(self):
        for name in ("NewtonActuatorDelayAPI", "NewtonActuatorBacklashAPI"):
            with self.subTest(name=name):
                prim = self.stage.DefinePrim(f"/{name}", "NewtonActuator")
                prim.ApplyAPI(name)
                self.assertTrue(prim.HasAPI("NewtonActuatorInputProcessorBaseAPI"))

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonActuatorInputProcessorBaseAPI"))


class TestNewtonActuatorDelayAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuatorDelayAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuatorDelayAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuatorDelayAPI")
        self.assertEqual(schema_type, "NewtonActuatorDelayAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonActuatorDelayAPI"))
        self.prim.ApplyAPI("NewtonActuatorDelayAPI")
        self.assertTrue(self.prim.HasAPI("NewtonActuatorDelayAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:delaySteps"))

    def test_delay_steps(self):
        self.prim.ApplyAPI("NewtonActuatorDelayAPI")
        attr = self.prim.GetAttribute("newton:delaySteps")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), 1)

        attr.Set(0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), 0)

        attr.Set(5)
        self.assertEqual(attr.Get(), 5)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertEqual(hard.GetMinimum(), 0)
            self.assertIsNone(hard.GetMaximum())

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonActuatorDelayAPI"))


class TestNewtonActuatorBacklashAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuatorBacklashAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuatorBacklashAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuatorBacklashAPI")
        self.assertEqual(schema_type, "NewtonActuatorBacklashAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonActuatorBacklashAPI"))
        self.prim.ApplyAPI("NewtonActuatorBacklashAPI")
        self.assertTrue(self.prim.HasAPI("NewtonActuatorBacklashAPI"))
        self.assertTrue(self.prim.HasRelationship("newton:backlashJoint"))

    def test_backlash_joint_relationship(self):
        self.prim.ApplyAPI("NewtonActuatorBacklashAPI")
        rel = self.prim.GetRelationship("newton:backlashJoint")
        self.assertFalse(rel.HasAuthoredTargets())
        rel.SetTargets([Sdf.Path("/World/BacklashJoint")])
        targets = rel.GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(str(targets[0]), "/World/BacklashJoint")

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonActuatorBacklashAPI"))


class TestNewtonActuatorControlBaseAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuatorControlBaseAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuatorControlBaseAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuatorControlBaseAPI")
        self.assertEqual(schema_type, "NewtonActuatorControlBaseAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.prim.ApplyAPI("NewtonActuatorControlBaseAPI")
        self.assertTrue(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonActuatorControlBaseAPI"))


class TestNewtonPDControlAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsPDControlAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsPDControlAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsPDControlAPI")
        self.assertEqual(schema_type, "NewtonPDControlAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonPDControlAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.prim.ApplyAPI("NewtonPDControlAPI")
        self.assertTrue(self.prim.HasAPI("NewtonPDControlAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:kp"))
        self.assertTrue(self.prim.HasAttribute("newton:kd"))

    def test_kp(self):
        self.prim.ApplyAPI("NewtonPDControlAPI")
        attr = self.prim.GetAttribute("newton:kp")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(50.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 50.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_kd(self):
        self.prim.ApplyAPI("NewtonPDControlAPI")
        attr = self.prim.GetAttribute("newton:kd")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(5.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 5.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())


class TestNewtonPIDControlAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsPIDControlAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsPIDControlAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsPIDControlAPI")
        self.assertEqual(schema_type, "NewtonPIDControlAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonPIDControlAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.prim.ApplyAPI("NewtonPIDControlAPI")
        self.assertTrue(self.prim.HasAPI("NewtonPIDControlAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonPDControlAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:kp"))
        self.assertTrue(self.prim.HasAttribute("newton:kd"))
        self.assertTrue(self.prim.HasAttribute("newton:ki"))
        self.assertTrue(self.prim.HasAttribute("newton:integralMax"))

    def test_kp(self):
        self.prim.ApplyAPI("NewtonPDControlAPI")
        attr = self.prim.GetAttribute("newton:kp")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(50.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 50.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_kd(self):
        self.prim.ApplyAPI("NewtonPDControlAPI")
        attr = self.prim.GetAttribute("newton:kd")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(5.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 5.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_ki(self):
        self.prim.ApplyAPI("NewtonPIDControlAPI")
        attr = self.prim.GetAttribute("newton:ki")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(10.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 10.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_integral_max(self):
        self.prim.ApplyAPI("NewtonPIDControlAPI")
        attr = self.prim.GetAttribute("newton:integralMax")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), math.inf)

        attr.Set(100.0)
        self.assertAlmostEqual(attr.Get(), 100.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())


class TestNewtonNeuralControlAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsNeuralControlAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsNeuralControlAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsNeuralControlAPI")
        self.assertEqual(schema_type, "NewtonNeuralControlAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonNeuralControlAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.prim.ApplyAPI("NewtonNeuralControlAPI")
        self.assertTrue(self.prim.HasAPI("NewtonNeuralControlAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:modelPath"))

    def test_model_path(self):
        self.prim.ApplyAPI("NewtonNeuralControlAPI")
        attr = self.prim.GetAttribute("newton:modelPath")
        self.assertFalse(attr.HasAuthoredValue())

        attr.Set("path/to/model.pt")
        self.assertTrue(attr.HasAuthoredValue())
        self.assertEqual(attr.Get().path, "path/to/model.pt")

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonNeuralControlAPI"))


class TestNewtonBAMControlAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsBAMControlAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsBAMControlAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsBAMControlAPI")
        self.assertEqual(schema_type, "NewtonBAMControlAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonBAMControlAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        self.prim.ApplyAPI("NewtonBAMControlAPI")
        self.assertTrue(self.prim.HasAPI("NewtonBAMControlAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorControlBaseAPI"))
        for name in ("kp", "kt", "resistance", "vin", "maxPwm", "maxCurrent"):
            self.assertTrue(self.prim.HasAttribute(f"newton:{name}"), name)

    def test_required_attributes(self):
        self.prim.ApplyAPI("NewtonBAMControlAPI")
        for name, value, maximum in (
            ("kp", 8.0, None),
            ("kt", 0.6, None),
            ("resistance", 2.0, None),
            ("vin", 12.0, None),
            ("maxPwm", 0.9, 1.0),
        ):
            with self.subTest(name=name):
                attr = self.prim.GetAttribute(f"newton:{name}")
                self.assertFalse(attr.HasAuthoredValue())
                self.assertIsNone(attr.Get())

                attr.Set(value)
                self.assertTrue(attr.HasAuthoredValue())
                self.assertAlmostEqual(attr.Get(), value, places=6)

                if USD_HAS_LIMITS:
                    hard = attr.GetHardLimits()
                    self.assertTrue(hard.IsValid())
                    self.assertAlmostEqual(hard.GetMinimum(), 0.0)
                    if maximum is None:
                        self.assertIsNone(hard.GetMaximum())
                    else:
                        self.assertAlmostEqual(hard.GetMaximum(), maximum)

    def test_max_current(self):
        self.prim.ApplyAPI("NewtonBAMControlAPI")
        attr = self.prim.GetAttribute("newton:maxCurrent")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 0.0)

        attr.Set(1.5)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 1.5)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonBAMControlAPI"))


class TestNewtonActuatorClampingBaseAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsActuatorClampingBaseAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsActuatorClampingBaseAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsActuatorClampingBaseAPI")
        self.assertEqual(schema_type, "NewtonActuatorClampingBaseAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.prim.ApplyAPI("NewtonActuatorClampingBaseAPI")
        self.assertTrue(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonActuatorClampingBaseAPI"))


class TestNewtonMaxEffortClampingAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsMaxEffortClampingAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsMaxEffortClampingAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsMaxEffortClampingAPI")
        self.assertEqual(schema_type, "NewtonMaxEffortClampingAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonMaxEffortClampingAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.prim.ApplyAPI("NewtonMaxEffortClampingAPI")
        self.assertTrue(self.prim.HasAPI("NewtonMaxEffortClampingAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:maxEffort"))

    def test_max_effort(self):
        self.prim.ApplyAPI("NewtonMaxEffortClampingAPI")
        attr = self.prim.GetAttribute("newton:maxEffort")
        self.assertIsNotNone(attr)
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), math.inf)

        attr.Set(100.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 100.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonMaxEffortClampingAPI"))


class TestNewtonDCMotorClampingAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsDCMotorClampingAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsDCMotorClampingAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsDCMotorClampingAPI")
        self.assertEqual(schema_type, "NewtonDCMotorClampingAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonDCMotorClampingAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.prim.ApplyAPI("NewtonDCMotorClampingAPI")
        self.assertTrue(self.prim.HasAPI("NewtonDCMotorClampingAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonMaxEffortClampingAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:maxMotorEffort"))
        self.assertTrue(self.prim.HasAttribute("newton:saturationEffort"))
        self.assertTrue(self.prim.HasAttribute("newton:velocityLimit"))

    def test_max_motor_effort(self):
        self.prim.ApplyAPI("NewtonDCMotorClampingAPI")
        attr = self.prim.GetAttribute("newton:maxMotorEffort")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), math.inf)

        attr.Set(100.0)
        self.assertAlmostEqual(attr.Get(), 100.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_saturation_effort(self):
        self.prim.ApplyAPI("NewtonDCMotorClampingAPI")
        attr = self.prim.GetAttribute("newton:saturationEffort")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), math.inf)

        attr.Set(10.0)
        self.assertAlmostEqual(attr.Get(), 10.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_velocity_limit(self):
        self.prim.ApplyAPI("NewtonDCMotorClampingAPI")
        attr = self.prim.GetAttribute("newton:velocityLimit")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), math.inf)

        attr.Set(30.0)
        self.assertTrue(attr.HasAuthoredValue())
        self.assertAlmostEqual(attr.Get(), 30.0)

        if USD_HAS_LIMITS:
            hard = attr.GetHardLimits()
            self.assertTrue(hard.IsValid())
            self.assertAlmostEqual(hard.GetMinimum(), 0.0)
            self.assertIsNone(hard.GetMaximum())

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonDCMotorClampingAPI"))


class TestNewtonPositionBasedClampingAPI(unittest.TestCase):
    def setUp(self):
        self.stage: Usd.Stage = Usd.Stage.CreateInMemory()
        self.prim: Usd.Prim = self.stage.DefinePrim("/Actuator", "NewtonActuator")

    def test_api_registered(self):
        plug_type = Plug.Registry().FindTypeByName("NewtonPhysicsPositionBasedClampingAPI")
        self.assertEqual(plug_type.typeName, "NewtonPhysicsPositionBasedClampingAPI")
        schema_type = Usd.SchemaRegistry().GetSchemaTypeName("NewtonPhysicsPositionBasedClampingAPI")
        self.assertEqual(schema_type, "NewtonPositionBasedClampingAPI")

    def test_api_application(self):
        self.assertFalse(self.prim.HasAPI("NewtonPositionBasedClampingAPI"))
        self.assertFalse(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.prim.ApplyAPI("NewtonPositionBasedClampingAPI")
        self.assertTrue(self.prim.HasAPI("NewtonPositionBasedClampingAPI"))
        self.assertTrue(self.prim.HasAPI("NewtonActuatorClampingBaseAPI"))
        self.assertTrue(self.prim.HasAttribute("newton:lookupPositions"))
        self.assertTrue(self.prim.HasAttribute("newton:lookupEfforts"))

    def test_lookup_positions(self):
        self.prim.ApplyAPI("NewtonPositionBasedClampingAPI")
        attr = self.prim.GetAttribute("newton:lookupPositions")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), [])

        attr.Set([0.0, 45.0, 90.0])
        self.assertEqual(attr.Get(), [0.0, 45.0, 90.0])

    def test_lookup_efforts(self):
        self.prim.ApplyAPI("NewtonPositionBasedClampingAPI")
        attr = self.prim.GetAttribute("newton:lookupEfforts")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), [])

        attr.Set([100.0, 80.0, 50.0])
        self.assertEqual(attr.Get(), [100.0, 80.0, 50.0])

    def test_api_limitations(self):
        xform = self.stage.DefinePrim("/NotActuator", "Xform")
        self.assertFalse(xform.CanApplyAPI("NewtonPositionBasedClampingAPI"))


if __name__ == "__main__":
    unittest.main()
