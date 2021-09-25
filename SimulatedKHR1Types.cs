//-----------------------------------------------------------------------
//  This file is part of the Microsoft Robotics Studio Code Samples.
//
//  Copyright (C) Microsoft Corporation.  All rights reserved.
//
//  $File: SimulatedKHR1Types.cs $ $Revision: 4 $
//-----------------------------------------------------------------------

using Microsoft.Ccr.Core;
using Microsoft.Dss.ServiceModel.Dssp;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using W3C.Soap;

[assembly: ContractNamespace(Microsoft.Robotics.Services.KHR1.Simulated.Contract.Namespace, ClrNamespace = "Microsoft.Robotics.Services.KHR1.Simulated")]


namespace Microsoft.Robotics.Services.KHR1.Simulated
{
    
    public sealed class Contract
    {
        public const string Namespace = "http://schemas.microsoft.com/robotics/simulation/services/2006/07/simulatedkhr1.html";
        /// Prevent this class from being instantiated
        private Contract()
        {
        }
    }
}
