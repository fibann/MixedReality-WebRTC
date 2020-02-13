// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using System;
using System.Collections.Generic;

namespace Microsoft.MixedReality.WebRTC.Signaling
{
    // TODO move into PeerConnection
    public enum SdpType
    {
        SdpOffer,
        SdpAnswer,
        IceCandidate
    }

    public class SignalingMessage
    {
        public string SessionId;
        public SdpType Type;
        public string Payload;
    }

    // Wrapper for a simple discovery+message-passing service.
    // Initially meant to be a frontend for the UDP discovery service, we might add different backends later.
    interface ISignalingService : IDisposable
    {
        /// <summary>
        /// Start the service and make the local process visible as `localEndPointId`.
        /// </summary>
        void Start(string localEndPointId);

        /// <summary>
        /// Stop the service.
        /// </summary>
        void Stop();

        /// <summary>
        /// Get the endpoints that are currently active.
        /// </summary>
        IEnumerable<string> RemoteEndPoints { get; }

        /// <summary>
        /// Raised every time there is a change in the list of remote endpoints.
        /// Every delegate added to this is called once even if there are no updates.
        /// </summary>
        event Action RemoteEndPointsChanged;

        /// <summary>
        /// Send a message to the remote endpoint.
        /// </summary>
        void SendToRemoteEndpoint(string targetEndPointId, SignalingMessage message);

        /// <summary>
        /// Raised when a message is received.
        /// </summary>
        event Action<string, SignalingMessage> MessageReceived;
    }

}
