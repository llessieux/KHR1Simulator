//-----------------------------------------------------------------------
//  This file is part of the Microsoft Robotics Studio Code Samples.
//
//  Copyright (C) Microsoft Corporation.  All lefts reserved.
//
//  $File: SimulatedKHR1.cs $ $Revision: 11 $
//-----------------------------------------------------------------------

using Microsoft.Ccr.Core;
using Microsoft.Dss.Core;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using System;
using System.Collections.Generic;
using System.Security.Permissions;
using System.Runtime.Serialization;

using dx = Microsoft.DirectX;
using xml = System.Xml;
using arm = Microsoft.Robotics.Services.ArticulatedArm;
using simengine = Microsoft.Robotics.Simulation.Engine;
using Microsoft.Robotics.Simulation;
using Microsoft.Robotics.Simulation.Physics;
using Microsoft.Robotics.PhysicalModel;
using System.ComponentModel;
using Microsoft.Robotics.Simulation.Engine;
using W3C.Soap;

namespace Microsoft.Robotics.Services.KHR1.Simulated
{

    #region Robot platforms

    [DataContract]
    //[CLSCompliant(true)]
    public class MyBoxArmLinkEntity : VisualEntity
    {
        // we dont care to serialize the ArmLink entity internals
        // Instead we rely on a higher level entity to
        // set the joints and shapes when it deserializes.
        Joint _lowerJoint;
        public Joint LowerJoint
        {
            get { return _lowerJoint; }
            set { _lowerJoint = value; }
        }

        Joint _upperJoint;

        public Joint UpperJoint
        {
            get { return _upperJoint; }
            set { _upperJoint = value; }
        }

        Pose _visibleMeshPose;
        BoxShape _shape;

        public BoxShape Shape
        {
            get { return _shape; }
            set { _shape = value; }
        }

        public MyBoxArmLinkEntity() { }

        public MyBoxArmLinkEntity(string name, string mesh, Pose visibleMeshPose)
        {
            State.Name = name;
            State.Assets.Mesh = mesh;
            State.Assets.Effect = "KHR1.fx";
            //State.MassDensity.AngularDamping = 10000;
            //State.MassDensity.LinearDamping = 10000;
            _visibleMeshPose = visibleMeshPose;
        }

        public override void Initialize(Microsoft.DirectX.Direct3D.Device device, PhysicsEngine physicsEngine)
        {
            // assume the parent entity set the _shape field before initialize was called
            State.PhysicsPrimitives.Add(_shape);
            CreateAndInsertPhysicsEntity(physicsEngine);
            base.Initialize(device, physicsEngine);
            PhysicsEntity.SolverIterationCount = 64;
        }

        public override void Update(FrameUpdate update)
        {
            if (UpperJoint != null)
                ((PhysicsJoint)UpperJoint).UpdateState();
            if (LowerJoint != null)
                ((PhysicsJoint)LowerJoint).UpdateState();
            base.Update(update);
        }

        public override void Render(Microsoft.DirectX.Direct3D.Device device, MatrixTransforms transforms)
        {
            // We apply three position and orientation transforms before we can render the visible mesh
            // First rotate and translate by the visible mesh pose which positions the graphics mesh properly
            dx.Matrix world =
                dx.Matrix.RotationQuaternion(TypeConversion.ToDx(_visibleMeshPose.Orientation)) *
                dx.Matrix.Translation(TypeConversion.ToDx(_visibleMeshPose.Position));
            // Second, rotate and translate by the physics shape pose
            world *= dx.Matrix.RotationQuaternion(
                TypeConversion.ToDx(State.PhysicsPrimitives[0].State.LocalPose.Orientation)) *
                dx.Matrix.Translation(
                TypeConversion.ToDx(State.PhysicsPrimitives[0].State.LocalPose.Position));
            // Third, rotate and translate by the whole entity pose
            world *= dx.Matrix.RotationQuaternion(TypeConversion.ToDx(State.Pose.Orientation)) *
                dx.Matrix.Translation(TypeConversion.ToDx(State.Pose.Position));

            transforms.World = world;
            // now render
            Render(device, transforms, Meshes[0]);
        }
    }

    [DataContract]
    public class KHR1BodyEntity : MyBoxArmLinkEntity
    {
        // we dont care to serialize the ArmLink entity internals
        // Instead we rely on a higher level entity to
        // set the joints and shapes when it deserializes.

        Joint _rightShoulderJoint;
        Joint _leftShoulderJoint;
        Joint _rightHipJoint;
        Joint _leftHipJoint;

        Pose _visibleMeshPose;

        public Joint HeadJoint
        {
            get { return UpperJoint; }
            set { UpperJoint = value; }
        }
        public Joint RightShoulderJoint
        {
            get { return _rightShoulderJoint; }
            set { _rightShoulderJoint = value; }
        }
        public Joint LeftShoulderJoint
        {
            get { return _leftShoulderJoint; }
            set { _leftShoulderJoint = value; }
        }
        public Joint RightHipJoint
        {
            get { return _rightHipJoint; }
            set { _rightHipJoint = value; }
        }
        public Joint LeftHipJoint
        {
            get { return _leftHipJoint; }
            set { _leftHipJoint = value; }
        }

        public KHR1BodyEntity() { }

        public KHR1BodyEntity(string name, string mesh, Pose visibleMeshPose)
        {
            State.Name = name;
            State.Assets.Mesh = mesh;
            State.Assets.Effect = "KHR1.fx";
            //State.MassDensity.AngularDamping = 10000;
            //State.MassDensity.LinearDamping = 10000;
            _visibleMeshPose = visibleMeshPose;
        }

        public override void Update(FrameUpdate update)
        {
            if (UpperJoint != null)
                ((PhysicsJoint)UpperJoint).UpdateState();
            if (RightShoulderJoint != null)
                ((PhysicsJoint)RightShoulderJoint).UpdateState();
            if (LeftShoulderJoint != null)
                ((PhysicsJoint)LeftShoulderJoint).UpdateState();
            if (RightHipJoint != null)
                ((PhysicsJoint)RightHipJoint).UpdateState();
            if (LeftHipJoint != null)
                ((PhysicsJoint)LeftHipJoint).UpdateState();

            base.Update(update);
        }

        public override void Render(Microsoft.DirectX.Direct3D.Device device, MatrixTransforms transforms)
        {
            // We apply three position and orientation transforms before we can render the visible mesh
            // First rotate and translate by the visible mesh pose which positions the graphics mesh properly
            dx.Matrix world =
                dx.Matrix.RotationQuaternion(TypeConversion.ToDx(_visibleMeshPose.Orientation)) *
                dx.Matrix.Translation(TypeConversion.ToDx(_visibleMeshPose.Position));
            // Second, rotate and translate by the physics shape pose
            world *= dx.Matrix.RotationQuaternion(
                TypeConversion.ToDx(State.PhysicsPrimitives[0].State.LocalPose.Orientation)) *
                dx.Matrix.Translation(
                TypeConversion.ToDx(State.PhysicsPrimitives[0].State.LocalPose.Position));
            // Third, rotate and translate by the whole entity pose
            world *= dx.Matrix.RotationQuaternion(TypeConversion.ToDx(State.Pose.Orientation)) *
                dx.Matrix.Translation(TypeConversion.ToDx(State.Pose.Position));

            transforms.World = world;
            // now render
            Render(device, transforms, Meshes[0]);
        }
    }

    [DataContract]
    //[CLSCompliant(true)]
    public class Kondo_KHR1 : Microsoft.Robotics.Simulation.Engine.VisualEntity
    {
        const int _jointCount = 17;

        //[DataMember]
        public int JointCount
        {
            get { return _jointCount; }
        }

        List<Joint> _joints = new List<Joint>();
        //[DataMember]
        public List<Joint> Joints
        {
            get { return _joints; }
            set { _joints = value; }
        }

        BoxShape _bodyShape;
        BoxShape [] _armShape;

        public Kondo_KHR1() { }

        public Kondo_KHR1(Vector3 position)
        {
            State.Pose.Position = position;

            //BBox dimensions of the various parts, starting with the Head....
            //The body is not included here
            float [] dims = {   0.43999998f,0.49399994f,0.20899998f,
                                0.22000000f,0.52750000f,0.36455000f,
                                0.23499999f,0.54025024f,0.77377707f ,
                                0.40858700f,0.54025024f,0.82992104f ,
                                0.21999999f,0.52750000f,0.36455002f ,
                                0.23499999f,0.54024994f,0.77377991f ,
                                0.40858801f,0.54024994f,0.82991989f ,
                                0.53374998f,0.63865997f,0.52754496f ,
                                0.46883199f,0.80168999f,0.54231298f ,
                                0.26883520f,0.78048401f,0.54037500f ,
                                0.52750100f,0.63830032f,0.52754599f ,
                                0.92550327f,0.27022003f,0.48050823f ,
                                0.53374998f,0.63776993f,0.52751396f ,
                                0.46883198f,0.80165001f,0.54097801f ,
                                0.26883432f,0.77931602f,0.54029694f ,
                                0.52750105f,0.63797028f,0.52751297f ,
                                0.92549999f,0.27086871f,0.48052582f  };

            float[] estimate_weights = {
                0.05f,
                0.015f,0.07f,0.07f,
                0.015f,0.07f,0.07f,
                0.015f,0.12f,0.07f,0.015f,0.09f,
                0.015f,0.12f,0.07f,0.015f,0.09f
            };
            _armShape = new BoxShape[_jointCount];
           
            float reductionFactor = 0.75f;
            for(int i=0;i<_jointCount;i++)
            {
                _armShape[i] = new BoxShape(
                    new BoxShapeProperties(estimate_weights[i],
                    new Pose(new Vector3(0, 0, 0)),
                    //Take a slightly smaller bbox to allow proper movements.
                    new Vector3(dims[i * 3] * reductionFactor, dims[i * 3 + 1] * reductionFactor, dims[i * 3 + 2] * reductionFactor)
                    ));

            }

            // all joints will be essentially of the same type:
            // A single DOF free, around the Joint Axis. A drive will be associated with
            // that DOF. Each joint will have a different joint axis and joint normal
            for (int i = 0; i < _jointCount; i++)
            {
                JointAngularProperties commonAngular = new JointAngularProperties();
                commonAngular.TwistMode = JointDOFMode.Free;
                
                commonAngular.TwistDrive = new JointDriveProperties(
                    JointDriveMode.Position,
                    new SpringProperties(50000, 10000, 0),
                    1000000);

                //Speed : 60 degrees in 0.17s, now where do i specify that.
                //Attempt at setting the speed.... 6.16f radians per second.I hope
                commonAngular.DriveTargetVelocity = new Vector3(6.16f, 6.16f, 6.16f);

                JointProperties j = new JointProperties(commonAngular, null, null);
                
                //Do not enable the collision until we have a more precise representation of the parts
                //j.EnableCollisions = true;

                _joints.Add(PhysicsJoint.Create(j));
            }

            // joints must be names
            _joints[0].State.Name = "Joint0 CH6 Head";
            _joints[1].State.Name = "Joint1 CH1 right Shoulder";
            _joints[2].State.Name = "Joint2 CH2 right Arm";
            _joints[3].State.Name = "Joint3 CH3 right Hand";
            _joints[4].State.Name = "Joint4 CH7 left Shoulder";
            _joints[5].State.Name = "Joint5 CH8 left Arm";
            _joints[6].State.Name = "Joint6 CH9 left Hand";
            _joints[7].State.Name = "Joint7 CH13 right Hip";
            _joints[8].State.Name = "Joint8 CH14 right Leg1";
            _joints[9].State.Name = "Joint9 CH15 right Leg2";
            _joints[10].State.Name = "Joint10 CH16 right Ankle";
            _joints[11].State.Name = "Joint11 CH17 right Foot";
            _joints[12].State.Name = "Joint12 CH19 left Hip";
            _joints[13].State.Name = "Joint13 CH20 left Leg1";
            _joints[14].State.Name = "Joint14 CH21 left Leg2";
            _joints[15].State.Name = "Joint15 CH22 left Ankle";
            _joints[16].State.Name = "Joint16 CH23 left Foot";

            // programmatically build articulated arm
            CreateBody(_joints[0], _joints[1], _joints[4], _joints[7], _joints[12]);
            CreateHead(_joints[0]);

            CreaterightShoulder(_joints[1]);
            CreaterightArm(_joints[2]);
            CreaterightHand(_joints[3]);

            CreateleftShoulder(_joints[4]);
            CreateleftArm(_joints[5]);
            CreateleftHand(_joints[6]);

            CreaterightHip(_joints[7]);
            CreaterightLeg1(_joints[8]);
            CreaterightLeg2(_joints[9]);
            CreaterightAnkle(_joints[10]);
            CreaterightFoot(_joints[11]);

            CreateleftHip(_joints[12]);
            CreateleftLeg1(_joints[13]);
            CreateleftLeg2(_joints[14]);
            CreateleftAnkle(_joints[15]);
            CreateleftFoot(_joints[16]);
        }

        #region Link creation

        void CreateBody(Joint joint0, Joint joint1, Joint joint2, Joint joint3, Joint joint4)
        {
            float mass = 0.22f; //Estimate weight of the body

            _bodyShape = new BoxShape(
                new BoxShapeProperties(mass, 
                new Pose(new Vector3(0, 0, 0)),
                new Vector3(0.516055f, 1.080760f, 1.150502f)
                ));

            PhysicsJoint headJoint = (PhysicsJoint)joint0;
            headJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0), // joint rotates around the Y(vertical axis)
                new Vector3(-0.00176299f, 0.41175247f, -0.00217850f));

            PhysicsJoint rightShoulderJoint = (PhysicsJoint)joint1;
            rightShoulderJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.03957299f , 0.27377243f , -0.55985849f));

            PhysicsJoint leftShoulderJoint = (PhysicsJoint)joint2;
            leftShoulderJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.03957299f, 0.27561234f, 0.55928146f ));

            PhysicsJoint rightHipJoint = (PhysicsJoint)joint3;
            rightHipJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0), // joint rotates around the X(Front/Back axis)
                new Vector3(-0.06253299f , -0.39792770f , -0.23489849f ));

            PhysicsJoint leftHipJoint = (PhysicsJoint)joint4;
            leftHipJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0), // joint rotates around the X(Front/Back axis)
                new Vector3(-0.00179299f , -0.39736771f , 0.22467148f ));

            KHR1BodyEntity link = new KHR1BodyEntity("Body",
               "khr1_Main_01.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 1, 0), 0))));

            link.HeadJoint = headJoint;
            link.RightShoulderJoint = rightShoulderJoint;
            link.LeftShoulderJoint = leftShoulderJoint;
            link.RightHipJoint = rightHipJoint;
            link.LeftHipJoint = leftHipJoint;

            Children.Add(link);
        }

        void CreateHead(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                new Vector3(-0.10388699f, -0.30339233f , 0.00228188f ));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH6 Head",
               "khr1_CH6_HEAD.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 1, 0), 0))));
            link.LowerJoint = joint;
            Children.Add(link);
        }

        void CreaterightShoulder(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.00519377f , 0.01233093f , 0.12117594f ));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH1 right Shoulder",
               "khr1_CH1_SHOULDER_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightArm(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                        new Vector3(-0.00518377f , 0.00148071f , -0.11299406f ));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                new Vector3(-0.00499631f , 0.05201538f , 0.26634252f ));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH2 right Arm",
               "khr1_CH2_ARM_Right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 1, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightHand(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                        new Vector3(-0.00119631f , 0.04636536f , -0.28293747f ));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                new Vector3(-0.04731522f , 0.07974426f , 0.13168625f ));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH3 right Hand",
               "khr1_CH3_Hand_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 1, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftShoulder(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.00519585f, 0.01787628f, -0.11158432f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH7 left Shoulder",
               "khr1_CH7_SHOULDER_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));
            link.LowerJoint = joint;
            

            Children.Add(link);
        }

        void CreateleftArm(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                        new Vector3(0.00102415f, 0.00086639f, 0.11553574f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                new Vector3(0.00121155f, 0.04965118f, -0.26413223f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH8 left Arm",
               "khr1_CH8_ARM_left.x",
               new Pose(new Vector3(0, 0, 0),TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0,1,0),0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftHand(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                        new Vector3(-0.00102845f, 0.04963104f, 0.29047775f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(0, 1, 0),// joint rotates around the Y(vertical axis)
                new Vector3(-0.04714776f, 0.07739319f, -0.12978683f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH9 left Hand",
               "khr1_CH9_HAND_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 1, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightHip(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                new Vector3(-0.05503141f, 0.21117706f, 0.01926224f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH13 right Hip",
               "khr1_CH13_HIP_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(1, 0, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightLeg1(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                        new Vector3(-0.00574141f, -0.22642288f, -0.01393776f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(0.00275999f, 0.25314102f, -0.08800594f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH14 right Leg1",
               "khr1_CH14_LEG1_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightLeg2(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.00151001f, -0.24973900f, -0.08797596f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(0.02230636f, 0.29085770f, -0.03452579f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH15 right Leg2",
               "khr1_CH15_LEG2_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightAnkle(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                        new Vector3(-0.02045364f, -0.26306232f, -0.02598579f));
            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.00524155f, 0.21952991f, -0.00723316f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH16 right Ankle",
               "khr1_CH16_ANKLE_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreaterightFoot(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                        new Vector3(-0.00355154f, -0.21813007f, 0.02183685f));
            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                new Vector3(-0.09462248f, 0.05270990f, 0.07731091f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH17 right Foot",
               "khr1_CH17_FOOT_right.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(1, 0, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftHip(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                new Vector3(0.00570860f, 0.21165375f, -0.02089569f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH19 left Hip",
               "khr1_CH19_HIP_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(1, 0, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftLeg1(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                        new Vector3(-0.00450140f, -0.22841629f, 0.01070435f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(0.00399997f, 0.25151382f, 0.08229614f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH20 left Leg1",
               "khr1_CH20_LEG1_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftLeg2(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                        new Vector3(-0.00096003f, -0.24794609f, 0.08663612f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(0.02285626f, 0.29235687f, 0.03035072f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH21 left Leg2",
               "khr1_CH21_LEG2_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftAnkle(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                        new Vector3(-0.01874373f, -0.25965313f, 0.02844070f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),// joint rotates around the Z (Left/Right of the robot)
                new Vector3(-0.00353171f, 0.22283459f, 0.00716911f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH22 left Ankle",
               "khr1_CH22_ANKLE_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(0, 0, 1), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }

        void CreateleftFoot(Joint joint)
        {
            PhysicsJoint commonJoint = (PhysicsJoint)joint;
            commonJoint.State.Connectors[0] = new EntityJointConnector(null,
                        new Vector3(0, 0, 1),
                        new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                        new Vector3(-0.00355171f, -0.21826542f, -0.02040085f));

            commonJoint.State.Connectors[1] = new EntityJointConnector(null,
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0),// joint rotates around the X(Front/Back axis)
                new Vector3(-0.09462066f, 0.05089790f, -0.07812508f));

            MyBoxArmLinkEntity link = new MyBoxArmLinkEntity("CH23 left Foot",
               "khr1_CH23_FOOT_left.x",
               new Pose(new Vector3(0, 0, 0), TypeConversion.FromDx(dx.Quaternion.RotationAxis(new dx.Vector3(1, 0, 0), 0))));

            link.LowerJoint = joint;
            
            Children.Add(link);
        }


        #endregion

        public override void Initialize(Microsoft.DirectX.Direct3D.Device device, PhysicsEngine physicsEngine)
        {
            // set the physics shape on each child entity
            for (int i = 1; i < Children.Count; i++)
            {
                ((MyBoxArmLinkEntity)Children[i]).Shape = _armShape[i-1];
            }
            ((MyBoxArmLinkEntity)Children[0]).Shape = _bodyShape;

            // initialize will load the graphics mesh and also initialize all our
            // children
            base.Initialize(device, physicsEngine);
            base.State.MassDensity.Mass = 1.2f;
            base.State.MassDensity.CenterOfMass = new Pose(new Vector3(0, 0.15f, 0));

            KHR1BodyEntity body = (KHR1BodyEntity)Children[0];

            body.HeadJoint.State.Connectors[1].Entity = Children[1];
            body.RightShoulderJoint.State.Connectors[1].Entity = Children[2];
            body.LeftShoulderJoint.State.Connectors[1].Entity = Children[5];
            body.RightHipJoint.State.Connectors[1].Entity = Children[8];
            body.LeftHipJoint.State.Connectors[1].Entity = Children[13];

            MyBoxArmLinkEntity link = Children[1] as MyBoxArmLinkEntity;
            link.LowerJoint.State.Connectors[0].Entity = body;

            link = Children[2] as MyBoxArmLinkEntity;
            link.LowerJoint.State.Connectors[0].Entity = body;
            link = Children[5] as MyBoxArmLinkEntity;
            link.LowerJoint.State.Connectors[0].Entity = body;
            link = Children[8] as MyBoxArmLinkEntity;
            link.LowerJoint.State.Connectors[0].Entity = body;
            link = Children[13] as MyBoxArmLinkEntity;
            link.LowerJoint.State.Connectors[0].Entity = body;

            int[] starts = { 2, 5, 8, 13 };
            int[] ends = { 4, 7, 12, 17 };
            for (int j = 0; j < 4; j++)
            {
                int limit = ends[j];
                for (int i = starts[j]; i <= limit; i++)
                {
                    link = Children[i] as MyBoxArmLinkEntity;
                    if (i < limit)
                    {
                        link.UpperJoint = _joints[i];
                    }

                    // set the entity name in upper connector
                    if (link.UpperJoint != null)
                    {
                        if ( link.UpperJoint.State.Connectors[0].Entity == null )
                            link.UpperJoint.State.Connectors[0].Entity = link;
                    }
                    if (link.LowerJoint != null)
                    {
                        if (link.LowerJoint.State.Connectors[1].Entity == null)
                            link.LowerJoint.State.Connectors[1].Entity = link;
                    }
                }
            }

            // second pass inserts joints into the simulation, which initializes and activates them
            // We have to do this after the joint connectors are all attached to entities
            for (int i = 0; i < _jointCount; i++)
            {
                physicsEngine.InsertJoint((PhysicsJoint)_joints[i]);
            }
        }

        /// <summary>
        /// Sets orientation only for angular drives
        /// </summary>
        /// <param name="j"></param>
        /// <param name="axisAngle"></param>
        public void SetJointTargetOrientation(Joint j, AxisAngle axisAngle)
        {
            Task<Joint, AxisAngle> deferredTask = new Task<Joint, AxisAngle>(j, axisAngle, SetJointTargetOrientationInternal);
            DeferredTaskQueue.Post(deferredTask);
        }

        /// <summary>
        /// Sets position and orientation depending on the DOF configuration of the joint
        /// </summary>
        /// <param name="j"></param>
        /// <param name="pose"></param>
        public void SetJointTargetPose(Joint j, Pose pose)
        {
            Task<Joint, Pose> deferredTask = new Task<Joint, Pose>(j, pose, SetJointTargetPoseInternal);
            DeferredTaskQueue.Post(deferredTask);
        }

        /// <summary>
        /// Sets angular or linear velocity
        /// </summary>
        /// <param name="j"></param>
        /// <param name="velocity"></param>
        public void SetJointTargetVelocity(Joint j, Vector3 velocity)
        {
            Task<Joint, Vector3> deferredTask = new Task<Joint, Vector3>(j, velocity, SetJointTargetVelocityInternal);
            DeferredTaskQueue.Post(deferredTask);
        }

        void SetJointTargetPoseInternal(Joint j, Pose pose)
        {
            if (j.State.Linear != null)
            {
                ((PhysicsJoint)j).SetLinearDrivePosition(pose.Position);
            }

            if (j.State.Angular != null)
            {
                ((PhysicsJoint)j).SetAngularDriveOrientation(pose.Orientation);
            }
        }

        void SetJointTargetOrientationInternal(Joint j, AxisAngle axisAngle)
        {
            if (j.State.Angular != null)
            {
                Quaternion target =
                    TypeConversion.FromDx(
                    dx.Quaternion.RotationAxis(TypeConversion.ToDx(axisAngle.Axis), axisAngle.Angle)
                    );
                ((PhysicsJoint)j).SetAngularDriveOrientation(target);
            }
        }

        void SetJointTargetVelocityInternal(Joint j, Vector3 velocity)
        {
            if (j.State.Linear != null)
            {
                ((PhysicsJoint)j).SetLinearDriveVelocity(velocity);
            }
            else
            {
                ((PhysicsJoint)j).SetAngularDriveVelocity(velocity);
            }
        }

    }

    #endregion

    [AlternateContract(arm.Contract.Namespace)]
    [Contract(Contract.Namespace)]
    [PermissionSet(SecurityAction.PermitOnly, Name="Execution")]
    public class SimulatedKHR1 : DsspServiceBase
    {
        #region Simulation Variables
        Kondo_KHR1 _entity;
        simengine.SimulationEnginePort _simEngine;
        PhysicsEngine _physicsEngine;
        #endregion

        private arm.ArticulatedArmState _state = new arm.ArticulatedArmState();
        Dictionary<string, Joint> _jointLookup;

        [ServicePort("/SimulatedKHR1", AllowMultipleInstances=true)]
        private arm.ArticulatedArmOperations _mainPort = new arm.ArticulatedArmOperations();
        /// <summary>
        /// Default Service Constructor
        /// </summary>
        public SimulatedKHR1(DsspServiceCreationPort creationPort) : 
                base(creationPort)
        {
			CreateSuccess();
        }
        /// <summary>
        /// Service Start
        /// </summary>
        protected override void Start()
        {
            // Listen on the main port for requests and call the appropriate handler.
            ActivateDsspOperationHandlers();

            // Publish the service to the local Node Directory
            DirectoryInsert();

			// display HTTP service Uri
			LogInfo(LogGroups.Console, "Service uri: ");

            // simulation specific init
            SpawnIterator(InitializeSimulation);
        }

        IEnumerator<ITask> InitializeSimulation()
        {
            _simEngine = simengine.SimulationEngine.GlobalInstancePort;
            _physicsEngine = PhysicsEngine.GlobalInstance;

            simengine.SimulationEnginePort notificationTarget = new simengine.SimulationEnginePort();

            // Subscribe to simulation engine, for our Kondo KHR1 entity
            _simEngine.Subscribe(ServiceInfo.PartnerList, notificationTarget);

            // wait for notification, timeout if nothing received
            yield return Arbiter.Choice(
                Arbiter.Receive(false,
                    TimeoutPort(DsspOperation.DefaultLongTimeSpan),
                    delegate(DateTime dt) { 
                        LogError("Timeout waiting for visual entity"); Shutdown(); 
                    }),
                Arbiter.Receive<simengine.InsertSimulationEntity>(false,
                    notificationTarget,
                    delegate(simengine.InsertSimulationEntity ins)
                    {
                        _entity = (Kondo_KHR1)ins.Body;                        
                    })
            );

            if (_entity == null)
                yield break;

            _state = new arm.ArticulatedArmState();
            // use the entity state as our state
            _state.Joints = _entity.Joints;
            // create dictionary for quick lookup of joints from name
            _jointLookup = new Dictionary<string, Joint>();
            foreach (Joint j in _state.Joints)
            {
                _jointLookup.Add(j.State.Name, j);
            }
        }

        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public virtual IEnumerator<ITask> GetHandler(arm.Get get)
        {
            get.ResponsePort.Post(_state);
            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public virtual IEnumerator<ITask> SetJointTargetPose(arm.SetJointTargetPose update)
        {
            Joint j = _jointLookup[update.Body.JointName];
            _entity.SetJointTargetOrientation(j, update.Body.TargetOrientation);
            update.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            yield break;
        }

        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public virtual IEnumerator<ITask> SetJointTargetVelocity(arm.SetJointTargetVelocity update)
        {
            Joint j = _jointLookup[update.Body.JointName];
            _entity.SetJointTargetVelocity(j, update.Body.TargetVelocity);
            update.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            yield break;
        }
    }
}
