using System;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.Core.Transforms;

[assembly: ServiceDeclaration(DssServiceDeclaration.Transform, SourceAssemblyKey = @"SimulatedKHR1.Y2006.M07, Version=0.0.0.0, Culture=neutral, PublicKeyToken=adf1b610070c42f4")]
[assembly: System.Security.AllowPartiallyTrustedCallers()]

namespace Dss.Transforms.TransformSimulatedKHR1
{

    public class Transforms: TransformBase
    {

        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_Kondo_KHR1_Microsoft_Robotics_Services_KHR1_Simulated_Kondo_KHR1(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.Kondo_KHR1 target = new Microsoft.Robotics.Services.KHR1.Simulated.Kondo_KHR1();
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.Kondo_KHR1 from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.Proxy.Kondo_KHR1;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.EntityState)Transform_Microsoft_Robotics_Simulation_Proxy_EntityState_Microsoft_Robotics_Simulation_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_Kondo_KHR1_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_Kondo_KHR1(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.Kondo_KHR1 target = new Microsoft.Robotics.Services.KHR1.Simulated.Proxy.Kondo_KHR1();
            Microsoft.Robotics.Services.KHR1.Simulated.Kondo_KHR1 from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.Kondo_KHR1;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.Proxy.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.Proxy.EntityState)Transform_Microsoft_Robotics_Simulation_EntityState_Microsoft_Robotics_Simulation_Proxy_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_MyBoxArmLinkEntity_Microsoft_Robotics_Services_KHR1_Simulated_MyBoxArmLinkEntity(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.MyBoxArmLinkEntity target = new Microsoft.Robotics.Services.KHR1.Simulated.MyBoxArmLinkEntity();
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.MyBoxArmLinkEntity from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.Proxy.MyBoxArmLinkEntity;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.EntityState)Transform_Microsoft_Robotics_Simulation_Proxy_EntityState_Microsoft_Robotics_Simulation_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_MyBoxArmLinkEntity_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_MyBoxArmLinkEntity(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.MyBoxArmLinkEntity target = new Microsoft.Robotics.Services.KHR1.Simulated.Proxy.MyBoxArmLinkEntity();
            Microsoft.Robotics.Services.KHR1.Simulated.MyBoxArmLinkEntity from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.MyBoxArmLinkEntity;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.Proxy.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.Proxy.EntityState)Transform_Microsoft_Robotics_Simulation_EntityState_Microsoft_Robotics_Simulation_Proxy_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_KHR1BodyEntity_Microsoft_Robotics_Services_KHR1_Simulated_KHR1BodyEntity(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.KHR1BodyEntity target = new Microsoft.Robotics.Services.KHR1.Simulated.KHR1BodyEntity();
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.KHR1BodyEntity from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.Proxy.KHR1BodyEntity;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.EntityState)Transform_Microsoft_Robotics_Simulation_Proxy_EntityState_Microsoft_Robotics_Simulation_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Services_KHR1_Simulated_KHR1BodyEntity_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_KHR1BodyEntity(object transformObj)
        {
            Microsoft.Robotics.Services.KHR1.Simulated.Proxy.KHR1BodyEntity target = new Microsoft.Robotics.Services.KHR1.Simulated.Proxy.KHR1BodyEntity();
            Microsoft.Robotics.Services.KHR1.Simulated.KHR1BodyEntity from = transformObj as Microsoft.Robotics.Services.KHR1.Simulated.KHR1BodyEntity;
            target.Flags = (Microsoft.Robotics.Simulation.Engine.Proxy.VisualEntityProperties)((System.Int32)from.Flags);
            target.State = (from.State == null) ? null : (Microsoft.Robotics.Simulation.Proxy.EntityState)Transform_Microsoft_Robotics_Simulation_EntityState_Microsoft_Robotics_Simulation_Proxy_EntityState(from.State);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_Proxy_EntityState_Microsoft_Robotics_Simulation_EntityState(object transformObj)
        {
            Microsoft.Robotics.Simulation.EntityState target = new Microsoft.Robotics.Simulation.EntityState();
            Microsoft.Robotics.Simulation.Proxy.EntityState from = transformObj as Microsoft.Robotics.Simulation.Proxy.EntityState;
            target.Name = from.Name;
            target.Assets = (from.Assets == null) ? null : (Microsoft.Robotics.Simulation.RenderingAssets)Transform_Microsoft_Robotics_Simulation_Proxy_RenderingAssets_Microsoft_Robotics_Simulation_RenderingAssets(from.Assets);
            target.Pose = (from.Pose == null) ? null : (Microsoft.Robotics.PhysicalModel.Pose)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Pose_Microsoft_Robotics_PhysicalModel_Pose(from.Pose);
            target.Velocity = (Microsoft.Robotics.PhysicalModel.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Vector3_Microsoft_Robotics_PhysicalModel_Vector3(from.Velocity);
            target.MassDensity = (from.MassDensity == null) ? null : (Microsoft.Robotics.Simulation.Physics.MassDensity)Transform_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity_Microsoft_Robotics_Simulation_Physics_MassDensity(from.MassDensity);
            target.Flags = (Microsoft.Robotics.Simulation.Physics.EntitySimulationModifiers)((System.Int32)from.Flags);
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_EntityState_Microsoft_Robotics_Simulation_Proxy_EntityState(object transformObj)
        {
            Microsoft.Robotics.Simulation.Proxy.EntityState target = new Microsoft.Robotics.Simulation.Proxy.EntityState();
            Microsoft.Robotics.Simulation.EntityState from = transformObj as Microsoft.Robotics.Simulation.EntityState;
            target.Name = from.Name;
            target.Assets = (from.Assets == null) ? null : (Microsoft.Robotics.Simulation.Proxy.RenderingAssets)Transform_Microsoft_Robotics_Simulation_RenderingAssets_Microsoft_Robotics_Simulation_Proxy_RenderingAssets(from.Assets);
            target.Pose = (from.Pose == null) ? null : (Microsoft.Robotics.PhysicalModel.Proxy.Pose)Transform_Microsoft_Robotics_PhysicalModel_Pose_Microsoft_Robotics_PhysicalModel_Proxy_Pose(from.Pose);
            target.Velocity = (Microsoft.Robotics.PhysicalModel.Proxy.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Vector3_Microsoft_Robotics_PhysicalModel_Proxy_Vector3(from.Velocity);
            target.MassDensity = (from.MassDensity == null) ? null : (Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity)Transform_Microsoft_Robotics_Simulation_Physics_MassDensity_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity(from.MassDensity);
            target.Flags = (Microsoft.Robotics.Simulation.Physics.Proxy.EntitySimulationModifiers)((System.Int32)from.Flags);
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Proxy_Vector3_Microsoft_Robotics_PhysicalModel_Vector3(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Vector3 target = new Microsoft.Robotics.PhysicalModel.Vector3();
            Microsoft.Robotics.PhysicalModel.Proxy.Vector3 from = (Microsoft.Robotics.PhysicalModel.Proxy.Vector3)transformObj;
            target.X = from.X;
            target.Y = from.Y;
            target.Z = from.Z;
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Vector3_Microsoft_Robotics_PhysicalModel_Proxy_Vector3(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Proxy.Vector3 target = new Microsoft.Robotics.PhysicalModel.Proxy.Vector3();
            Microsoft.Robotics.PhysicalModel.Vector3 from = (Microsoft.Robotics.PhysicalModel.Vector3)transformObj;
            target.X = from.X;
            target.Y = from.Y;
            target.Z = from.Z;
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Proxy_Pose_Microsoft_Robotics_PhysicalModel_Pose(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Pose target = new Microsoft.Robotics.PhysicalModel.Pose();
            Microsoft.Robotics.PhysicalModel.Proxy.Pose from = transformObj as Microsoft.Robotics.PhysicalModel.Proxy.Pose;
            target.Position = (Microsoft.Robotics.PhysicalModel.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Vector3_Microsoft_Robotics_PhysicalModel_Vector3(from.Position);
            target.Orientation = (Microsoft.Robotics.PhysicalModel.Quaternion)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion_Microsoft_Robotics_PhysicalModel_Quaternion(from.Orientation);
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Pose_Microsoft_Robotics_PhysicalModel_Proxy_Pose(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Proxy.Pose target = new Microsoft.Robotics.PhysicalModel.Proxy.Pose();
            Microsoft.Robotics.PhysicalModel.Pose from = transformObj as Microsoft.Robotics.PhysicalModel.Pose;
            target.Position = (Microsoft.Robotics.PhysicalModel.Proxy.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Vector3_Microsoft_Robotics_PhysicalModel_Proxy_Vector3(from.Position);
            target.Orientation = (Microsoft.Robotics.PhysicalModel.Proxy.Quaternion)Transform_Microsoft_Robotics_PhysicalModel_Quaternion_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion(from.Orientation);
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion_Microsoft_Robotics_PhysicalModel_Quaternion(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Quaternion target = new Microsoft.Robotics.PhysicalModel.Quaternion();
            Microsoft.Robotics.PhysicalModel.Proxy.Quaternion from = (Microsoft.Robotics.PhysicalModel.Proxy.Quaternion)transformObj;
            target.X = from.X;
            target.Y = from.Y;
            target.Z = from.Z;
            target.W = from.W;
            return target;
        }


        public static object Transform_Microsoft_Robotics_PhysicalModel_Quaternion_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion(object transformObj)
        {
            Microsoft.Robotics.PhysicalModel.Proxy.Quaternion target = new Microsoft.Robotics.PhysicalModel.Proxy.Quaternion();
            Microsoft.Robotics.PhysicalModel.Quaternion from = (Microsoft.Robotics.PhysicalModel.Quaternion)transformObj;
            target.X = from.X;
            target.Y = from.Y;
            target.Z = from.Z;
            target.W = from.W;
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_Proxy_RenderingAssets_Microsoft_Robotics_Simulation_RenderingAssets(object transformObj)
        {
            Microsoft.Robotics.Simulation.RenderingAssets target = new Microsoft.Robotics.Simulation.RenderingAssets();
            Microsoft.Robotics.Simulation.Proxy.RenderingAssets from = transformObj as Microsoft.Robotics.Simulation.Proxy.RenderingAssets;
            target.Mesh = from.Mesh;
            target.DefaultTexture = from.DefaultTexture;
            target.Effect = from.Effect;
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_RenderingAssets_Microsoft_Robotics_Simulation_Proxy_RenderingAssets(object transformObj)
        {
            Microsoft.Robotics.Simulation.Proxy.RenderingAssets target = new Microsoft.Robotics.Simulation.Proxy.RenderingAssets();
            Microsoft.Robotics.Simulation.RenderingAssets from = transformObj as Microsoft.Robotics.Simulation.RenderingAssets;
            target.Mesh = from.Mesh;
            target.DefaultTexture = from.DefaultTexture;
            target.Effect = from.Effect;
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity_Microsoft_Robotics_Simulation_Physics_MassDensity(object transformObj)
        {
            Microsoft.Robotics.Simulation.Physics.MassDensity target = new Microsoft.Robotics.Simulation.Physics.MassDensity();
            Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity from = transformObj as Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity;
            target.Mass = from.Mass;
            target.InertiaTensor = (Microsoft.Robotics.PhysicalModel.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Vector3_Microsoft_Robotics_PhysicalModel_Vector3(from.InertiaTensor);
            target.CenterOfMass = (from.CenterOfMass == null) ? null : (Microsoft.Robotics.PhysicalModel.Pose)Transform_Microsoft_Robotics_PhysicalModel_Proxy_Pose_Microsoft_Robotics_PhysicalModel_Pose(from.CenterOfMass);
            target.Density = from.Density;
            target.LinearDamping = from.LinearDamping;
            target.AngularDamping = from.AngularDamping;
            return target;
        }


        public static object Transform_Microsoft_Robotics_Simulation_Physics_MassDensity_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity(object transformObj)
        {
            Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity target = new Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity();
            Microsoft.Robotics.Simulation.Physics.MassDensity from = transformObj as Microsoft.Robotics.Simulation.Physics.MassDensity;
            target.Mass = from.Mass;
            target.InertiaTensor = (Microsoft.Robotics.PhysicalModel.Proxy.Vector3)Transform_Microsoft_Robotics_PhysicalModel_Vector3_Microsoft_Robotics_PhysicalModel_Proxy_Vector3(from.InertiaTensor);
            target.CenterOfMass = (from.CenterOfMass == null) ? null : (Microsoft.Robotics.PhysicalModel.Proxy.Pose)Transform_Microsoft_Robotics_PhysicalModel_Pose_Microsoft_Robotics_PhysicalModel_Proxy_Pose(from.CenterOfMass);
            target.Density = from.Density;
            target.LinearDamping = from.LinearDamping;
            target.AngularDamping = from.AngularDamping;
            return target;
        }

        static Transforms()
        {
            AddProxyTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.Proxy.Kondo_KHR1), Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_Kondo_KHR1_Microsoft_Robotics_Services_KHR1_Simulated_Kondo_KHR1);
            AddSourceTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.Kondo_KHR1), Transform_Microsoft_Robotics_Services_KHR1_Simulated_Kondo_KHR1_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_Kondo_KHR1);
            AddProxyTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.Proxy.MyBoxArmLinkEntity), Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_MyBoxArmLinkEntity_Microsoft_Robotics_Services_KHR1_Simulated_MyBoxArmLinkEntity);
            AddSourceTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.MyBoxArmLinkEntity), Transform_Microsoft_Robotics_Services_KHR1_Simulated_MyBoxArmLinkEntity_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_MyBoxArmLinkEntity);
            AddProxyTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.Proxy.KHR1BodyEntity), Transform_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_KHR1BodyEntity_Microsoft_Robotics_Services_KHR1_Simulated_KHR1BodyEntity);
            AddSourceTransform(typeof(Microsoft.Robotics.Services.KHR1.Simulated.KHR1BodyEntity), Transform_Microsoft_Robotics_Services_KHR1_Simulated_KHR1BodyEntity_Microsoft_Robotics_Services_KHR1_Simulated_Proxy_KHR1BodyEntity);
            AddProxyTransform(typeof(Microsoft.Robotics.Simulation.Proxy.EntityState), Transform_Microsoft_Robotics_Simulation_Proxy_EntityState_Microsoft_Robotics_Simulation_EntityState);
            AddSourceTransform(typeof(Microsoft.Robotics.Simulation.EntityState), Transform_Microsoft_Robotics_Simulation_EntityState_Microsoft_Robotics_Simulation_Proxy_EntityState);
            AddProxyTransform(typeof(Microsoft.Robotics.PhysicalModel.Proxy.Vector3), Transform_Microsoft_Robotics_PhysicalModel_Proxy_Vector3_Microsoft_Robotics_PhysicalModel_Vector3);
            AddSourceTransform(typeof(Microsoft.Robotics.PhysicalModel.Vector3), Transform_Microsoft_Robotics_PhysicalModel_Vector3_Microsoft_Robotics_PhysicalModel_Proxy_Vector3);
            AddProxyTransform(typeof(Microsoft.Robotics.PhysicalModel.Proxy.Pose), Transform_Microsoft_Robotics_PhysicalModel_Proxy_Pose_Microsoft_Robotics_PhysicalModel_Pose);
            AddSourceTransform(typeof(Microsoft.Robotics.PhysicalModel.Pose), Transform_Microsoft_Robotics_PhysicalModel_Pose_Microsoft_Robotics_PhysicalModel_Proxy_Pose);
            AddProxyTransform(typeof(Microsoft.Robotics.PhysicalModel.Proxy.Quaternion), Transform_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion_Microsoft_Robotics_PhysicalModel_Quaternion);
            AddSourceTransform(typeof(Microsoft.Robotics.PhysicalModel.Quaternion), Transform_Microsoft_Robotics_PhysicalModel_Quaternion_Microsoft_Robotics_PhysicalModel_Proxy_Quaternion);
            AddProxyTransform(typeof(Microsoft.Robotics.Simulation.Proxy.RenderingAssets), Transform_Microsoft_Robotics_Simulation_Proxy_RenderingAssets_Microsoft_Robotics_Simulation_RenderingAssets);
            AddSourceTransform(typeof(Microsoft.Robotics.Simulation.RenderingAssets), Transform_Microsoft_Robotics_Simulation_RenderingAssets_Microsoft_Robotics_Simulation_Proxy_RenderingAssets);
            AddProxyTransform(typeof(Microsoft.Robotics.Simulation.Physics.Proxy.MassDensity), Transform_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity_Microsoft_Robotics_Simulation_Physics_MassDensity);
            AddSourceTransform(typeof(Microsoft.Robotics.Simulation.Physics.MassDensity), Transform_Microsoft_Robotics_Simulation_Physics_MassDensity_Microsoft_Robotics_Simulation_Physics_Proxy_MassDensity);
        }
    }
}

