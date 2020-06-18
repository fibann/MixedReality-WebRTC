using Microsoft.MixedReality.WebRTC;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using TestAppUwp;
using Windows.ApplicationModel;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Media.Capture;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace TestMultiPc
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            _start_Click(this, null);
        }

        private void Setup(PeerConnection pc, NodeDssSignaler signaler, out Task connected)
        {
            signaler.HttpServerAddress = "http://192.168.0.45:3000/";

            signaler.OnMessage += async (NodeDssSignaler.Message obj) =>
            {
                switch (obj.MessageType)
                {
                    case NodeDssSignaler.Message.WireMessageType.Offer:
                        var message = new SdpMessage { Type = SdpMessageType.Offer, Content = obj.Data };
                        await pc.SetRemoteDescriptionAsync(message);
                        // If we get an offer, we immediately send an answer back once the offer is applied
                        pc.CreateAnswer();
                        break;

                    case NodeDssSignaler.Message.WireMessageType.Answer:
                        var x = new SdpMessage { Type = SdpMessageType.Answer, Content = obj.Data };
                        await pc.SetRemoteDescriptionAsync(x);
                        break;

                    case NodeDssSignaler.Message.WireMessageType.Ice:
                        // TODO - This is NodeDSS-specific
                        pc.AddIceCandidate(obj.ToIceCandidate());
                        break;

                    default:
                        throw new InvalidOperationException($"Unhandled signaler message type '{obj.MessageType}'");
                }
            };
            pc.LocalSdpReadytoSend += (SdpMessage message) =>
            {
                var dssMessage = NodeDssSignaler.Message.FromSdpMessage(message);
                signaler.SendMessageAsync(dssMessage);
            };
            pc.IceCandidateReadytoSend += (IceCandidate candidate) =>
            {
                var message = NodeDssSignaler.Message.FromIceCandidate(candidate);
                signaler.SendMessageAsync(message);
            };

            var connectedTcs = new TaskCompletionSource<bool>();
            pc.Connected += () =>
            {
                Trace.WriteLine($"PC {pc.Name} CONNECTED");
                connectedTcs.SetResult(true);
            };
            connected = connectedTcs.Task;
        }
        class Remote
        {
            public PeerConnection pc;
            public NodeDssSignaler signaler;
            public Task connected;
            public Transceiver audioT;
            public Transceiver videoT;
        };

        private const int _numRemotes = 6;

        private async Task Start1()
        {
            var localPc = new PeerConnection();
            localPc.Name = "LocalA";
            var localSignaler = new NodeDssSignaler();
            localSignaler.LocalPeerId = "LocalA";
            localSignaler.RemotePeerId = "LocalB";
            Setup(localPc, localSignaler, out Task localConnected);
            localSignaler.StartPollingAsync();

            await localPc.InitializeAsync();
            var localAT = localPc.AddTransceiver(MediaKind.Audio, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.Inactive });
            var localVT = localPc.AddTransceiver(MediaKind.Video, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.Inactive });
            var arch = System.Environment.GetEnvironmentVariable("PROCESSOR_ARCHITECTURE");
            var localVS = arch == "AMD64" ?
                new LocalVideoTrackSettings { trackName = "local_video", width = 640, height = 480, framerate = 30 } :
                new LocalVideoTrackSettings { trackName = "local_video", width = 896, height = 504, framerate = 30 };

            var localVTrack = await LocalVideoTrack.CreateFromDeviceAsync(localVS);
            localVT.LocalVideoTrack = localVTrack;
            localVT.DesiredDirection = Transceiver.Direction.SendOnly;
            var localATrack = await LocalAudioTrack.CreateFromDeviceAsync();
            localAT.LocalAudioTrack = localATrack;
            localAT.DesiredDirection = Transceiver.Direction.SendOnly;

            localVTrack.I420AVideoFrameReady += LocalVTrack_I420AVideoFrameReady;

            localPc.CreateOffer();



            var remotes = new Remote[_numRemotes];
            for (int i = 0; i < remotes.Length; ++i)
            {
                var remote = new Remote();
                remote.pc = new PeerConnection();
                remote.pc.Name = $"Remote{i}A";
                remote.signaler = new NodeDssSignaler();
                remote.signaler.LocalPeerId = $"Remote{i}A";
                remote.signaler.RemotePeerId = $"Remote{i}B";
                Setup(remote.pc, remote.signaler, out remote.connected);

                await remote.pc.InitializeAsync();
                remote.audioT = remote.pc.AddTransceiver(MediaKind.Audio, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.ReceiveOnly });
                remote.videoT = remote.pc.AddTransceiver(MediaKind.Video, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.ReceiveOnly });

                remote.signaler.StartPollingAsync();
                remote.pc.CreateOffer();

                remotes[i] = remote;
            }

            await localConnected;
            foreach (var remote in remotes)
            {
                await remote.connected;
            }
            Trace.WriteLine("ALL CONNECTED ON SIDE A");

            await Task.Delay(2000);
            foreach (var remote in remotes)
            {
                string name = remote.pc.Name;
                remote.pc.Close();
                remote.pc.Dispose();
                Trace.WriteLine($"REMOTE {name} CLOSED ON SIDE A");
                await Task.Delay(1000);
            }
            Trace.WriteLine("ALL REMOTES CLOSED ON SIDE A");

            await Task.Delay(3000);
            Trace.WriteLine("DISPOSING TRACKS");

            localAT.LocalAudioTrack = null;
            localATrack.Dispose();
            localVT.LocalVideoTrack = null;
            localVTrack.Dispose();
            Trace.WriteLine("TRACKS DISPOSED ON SIDE A");

            localPc.Close();
            localPc.Dispose();
            Trace.WriteLine("LOCAL CLOSED ON SIDE A");
        }

        private static unsafe void CustomI420AFrameCallback(in FrameRequest request)
        {
            var data = stackalloc byte[32 * 16 + 16 * 8 * 2];
            int k = 0;
            // Y plane (full resolution)
            for (int j = 0; j < 16; ++j)
            {
                for (int i = 0; i < 32; ++i)
                {
                    data[k++] = 0x7F;
                }
            }
            // U plane (halved chroma in both directions)
            for (int j = 0; j < 8; ++j)
            {
                for (int i = 0; i < 16; ++i)
                {
                    data[k++] = 0x30;
                }
            }
            // V plane (halved chroma in both directions)
            for (int j = 0; j < 8; ++j)
            {
                for (int i = 0; i < 16; ++i)
                {
                    data[k++] = 0xB2;
                }
            }
            var dataY = new IntPtr(data);
            var frame = new I420AVideoFrame
            {
                dataY = dataY,
                dataU = dataY + (32 * 16),
                dataV = dataY + (32 * 16) + (16 * 8),
                dataA = IntPtr.Zero,
                strideY = 32,
                strideU = 16,
                strideV = 16,
                strideA = 0,
                width = 32,
                height = 16
            };
            request.CompleteRequest(frame);
        }

        private async Task Start2()
        {
            var localPc = new PeerConnection();
            localPc.Name = "LocalB";
            var localSignaler = new NodeDssSignaler();
            localSignaler.LocalPeerId = "LocalB";
            localSignaler.RemotePeerId = "LocalA";
            Setup(localPc, localSignaler, out Task localConnected);

            await localPc.InitializeAsync();

            var AT = localPc.AddTransceiver(MediaKind.Audio, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.ReceiveOnly });
            var VT = localPc.AddTransceiver(MediaKind.Video, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.ReceiveOnly });

            localSignaler.StartPollingAsync();

            var remotes = new Remote[_numRemotes];
            for (int i = 0; i < remotes.Length; ++i)
            {
                var remote = new Remote();
                remote.pc = new PeerConnection();
                remote.pc.Name = $"Remote{i}B";
                remote.signaler = new NodeDssSignaler();
                remote.signaler.LocalPeerId = $"Remote{i}B";
                remote.signaler.RemotePeerId = $"Remote{i}A";
                Setup(remote.pc, remote.signaler, out remote.connected);

                await remote.pc.InitializeAsync();

                var localAT = localPc.AddTransceiver(MediaKind.Audio, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.Inactive });
                var localVT = localPc.AddTransceiver(MediaKind.Video, new TransceiverInitSettings { InitialDesiredDirection = Transceiver.Direction.Inactive });
                var source = ExternalVideoTrackSource.CreateFromI420ACallback(CustomI420AFrameCallback);
                var localVTrack = LocalVideoTrack.CreateFromExternalSource($"remote video {i}", source);
                localVT.LocalVideoTrack = localVTrack;
                localVT.DesiredDirection = Transceiver.Direction.SendOnly;
                var localATrack = await LocalAudioTrack.CreateFromDeviceAsync();
                localAT.LocalAudioTrack = localATrack;
                localAT.DesiredDirection = Transceiver.Direction.SendOnly;

                remote.signaler.StartPollingAsync();

                remotes[i] = remote;
            }

            await localConnected;
            foreach (var remote in remotes)
            {
                await remote.connected;
            }
            Trace.WriteLine("ALL CONNECTED ON SIDE B");


            //track.Dispose();
            //Trace.WriteLine("Track disposed");
            //thisPc.Dispose();
            //Trace.WriteLine("PeerConnection disposed");
        }

        private DateTime _lastLocalVFrameMsg = DateTime.Now;

        private void LocalVTrack_I420AVideoFrameReady(I420AVideoFrame frame)
        {
            var now = DateTime.Now;
            if (now - _lastLocalVFrameMsg > TimeSpan.FromSeconds(1))
            {
                _lastLocalVFrameMsg = now;
                Trace.WriteLine("Local video active");
            }
        }


        private async Task RequestMediaAccessAsync()
        {
            // Ensure that the UWP app was authorized to capture audio (cap:microphone)
            // or video (cap:webcam), otherwise the native plugin will fail.
            try
            {
                MediaCapture mediaAccessRequester = new MediaCapture();
                var mediaSettings = new MediaCaptureInitializationSettings
                {
                    AudioDeviceId = "",
                    VideoDeviceId = "",
                    StreamingCaptureMode = StreamingCaptureMode.AudioAndVideo,
                    PhotoCaptureSource = PhotoCaptureSource.VideoPreview
                };
                await mediaAccessRequester.InitializeAsync(mediaSettings);
            }
            catch (UnauthorizedAccessException uae)
            {
                //Logger.Log("Access to A/V denied, check app permissions: " + uae.Message);
                throw uae;
            }
            catch (Exception ex)
            {
                //Logger.Log("Failed to initialize A/V with unknown exception: " + ex.Message);
                throw ex;
            }
        }

        private static bool IsFirstInstance()
        {
            var firstInstance = AppInstance.FindOrRegisterInstanceForKey("{44CD414E-B604-482E-8CFD-A9E09076CABD}");
            return firstInstance.IsCurrentInstance;
        }

        private async void _start_Click(object sender, RoutedEventArgs e)
        {
            await RequestMediaAccessAsync();
            var arch = System.Environment.GetEnvironmentVariable("PROCESSOR_ARCHITECTURE");
            if (arch != "AMD64" || !IsFirstInstance())
            {
                await Start1();
            }
            else
            {
                await Start2();
            }
        }
    }
}
