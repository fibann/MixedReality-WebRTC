// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "audio_frame_observer.h"
#include "callback.h"
#include "data_channel.h"
#include "mrs_errors.h"
#include "refptr.h"
#include "tracked_object.h"
#include "video_frame_observer.h"

namespace webrtc {
    class RTCStatsReport;
}

namespace Microsoft::MixedReality::WebRTC {

class PeerConnection;
class LocalVideoTrack;
class ExternalVideoTrackSource;
class DataChannel;

struct BitrateSettings {
  std::optional<int> start_bitrate_bps;
  std::optional<int> min_bitrate_bps;
  std::optional<int> max_bitrate_bps;
};



// TODO move to different header
template <class T>
constexpr const char* GetStatsName() {
  static_assert("Not a stats type");
}

template <>
constexpr const char* GetStatsName<mrsDataChannelStats>() {
  return "DataChannelStats";
}

class StatsReport : public RefCountedBase {
 public:
  StatsReport(const webrtc::RTCStatsReport&);
  ~StatsReport();

  template <class T>
  std::vector<T> GetStatsObjects() const {
    std::vector<T> res;
    mrsStatsReportGetObjects(&report_, GetStatsName<T>(),
                             &GetObjectsCallback<T>, &res);
    return res;
  }

  private:
  template <class T>
   static void GetObjectsCallback(void* user_data, const void* stats_object) {
     auto& res = *static_cast<std::vector<T>*>(user_data);
     res.push_back(*static_cast<const T*>(stats_object));
  }

 private:
  const webrtc::RTCStatsReport& report_;
};

/// The PeerConnection class is the entry point to most of WebRTC.
/// It encapsulates a single connection between a local peer and a remote peer,
/// and hosts some critical events for signaling and video rendering.
///
/// The high level flow to establish a connection is as follow:
/// - Create a peer connection object from a factory with
/// PeerConnection::create().
/// - Register a custom callback to the various signaling events.
/// - Optionally add audio/video/data tracks. These can also be added after the
/// connection is established, but see remark below.
/// - Create a peer connection offer, or wait for the remote peer to send an
/// offer, and respond with an answer.
///
/// At any point, before or after the connection is initated (CreateOffer() or
/// CreateAnswer()) or established (RegisterConnectedCallback()), some audio,
/// video, and data tracks can be added to it, with the following notable
/// remarks and restrictions:
/// - Data tracks use the DTLS/SCTP protocol and are encrypted; this requires a
/// handshake to exchange encryption secrets. This exchange is only performed
/// during the initial connection handshake if at least one data track is
/// present. As a consequence, at least one data track needs to be added before
/// calling CreateOffer() or CreateAnswer() if the application ever need to use
/// data channels. Otherwise trying to add a data channel after that initial
/// handshake will always fail.
/// - Adding and removing any kind of tracks after the connection has been
/// initiated result in a RenegotiationNeeded event to perform a new track
/// negotitation, which requires signaling to be working. Therefore it is
/// recommended when this is known in advance to add tracks before starting to
/// establish a connection, to perform the first handshake with the correct
/// tracks offer/answer right away.
class PeerConnection : public TrackedObject {
 public:
  /// Create a new PeerConnection based on the given |config|.
  /// This serves as the constructor for PeerConnection.
  static MRS_API ErrorOr<RefPtr<PeerConnection>> create(
      const PeerConnectionConfiguration& config,
      mrsPeerConnectionInteropHandle interop_handle);

  /// Set the name of the peer connection.
  MRS_API virtual void SetName(std::string_view name) = 0;

  //
  // Signaling
  //

  /// Callback fired when a local SDP message is ready to be sent to the remote
  /// peer by the signalling solution. The callback parameters are:
  /// - The null-terminated type of the SDP message. Valid values are "offer",
  /// "answer", and "ice".
  /// - The null-terminated SDP message content.
  using LocalSdpReadytoSendCallback = Callback<const char*, const char*>;

  /// Register a custom LocalSdpReadytoSendCallback.
  virtual void RegisterLocalSdpReadytoSendCallback(
      LocalSdpReadytoSendCallback&& callback) noexcept = 0;

  /// Callback fired when a local ICE candidate message is ready to be sent to
  /// the remote peer by the signalling solution. The callback parameters are:
  /// - The null-terminated ICE message content.
  /// - The mline index.
  /// - The MID string value.
  using IceCandidateReadytoSendCallback =
      Callback<const char*, int, const char*>;

  /// Register a custom IceCandidateReadytoSendCallback.
  virtual void RegisterIceCandidateReadytoSendCallback(
      IceCandidateReadytoSendCallback&& callback) noexcept = 0;

  /// Callback fired when the state of the ICE connection changed.
  /// Note that the current implementation (m71) mixes the state of ICE and
  /// DTLS, so this does not correspond exactly to
  using IceStateChangedCallback = Callback<IceConnectionState>;

  /// Register a custom IceStateChangedCallback.
  virtual void RegisterIceStateChangedCallback(
      IceStateChangedCallback&& callback) noexcept = 0;

  /// Callback fired when the state of the ICE gathering changed.
  using IceGatheringStateChangedCallback = Callback<IceGatheringState>;

  /// Register a custom IceStateChangedCallback.
  virtual void RegisterIceGatheringStateChangedCallback(
      IceGatheringStateChangedCallback&& callback) noexcept = 0;

  /// Callback fired when some SDP negotiation needs to be initiated, often
  /// because some tracks have been added to or removed from the peer
  /// connection, to notify the remote peer of the change.
  /// Typically an implementation will call CreateOffer() when receiving this
  /// notification to initiate a new SDP exchange. Failing to do so will prevent
  /// the remote peer from being informed about track changes.
  using RenegotiationNeededCallback = Callback<>;

  /// Register a custom RenegotiationNeededCallback.
  virtual void RegisterRenegotiationNeededCallback(
      RenegotiationNeededCallback&& callback) noexcept = 0;

  /// Notify the WebRTC engine that an ICE candidate has been received.
  virtual bool MRS_API AddIceCandidate(const char* sdp_mid,
                                       const int sdp_mline_index,
                                       const char* candidate) noexcept = 0;

  /// Notify the WebRTC engine that an SDP offer message has been received.
  virtual bool MRS_API SetRemoteDescription(const char* type,
                                            const char* sdp) noexcept = 0;

  //
  // Connection
  //

  /// Callback fired when the peer connection is established.
  /// This guarantees that the handshake process has terminated successfully,
  /// but does not guarantee that ICE exchanges are done.
  using ConnectedCallback = Callback<>;

  /// Register a custom ConnectedCallback.
  virtual void RegisterConnectedCallback(
      ConnectedCallback&& callback) noexcept = 0;

  virtual mrsResult SetBitrate(const BitrateSettings& settings) noexcept = 0;

  /// Create an SDP offer to attempt to establish a connection with the remote
  /// peer. Once the offer message is ready, the LocalSdpReadytoSendCallback
  /// callback is invoked to deliver the message.
  virtual bool MRS_API CreateOffer() noexcept = 0;

  /// Create an SDP answer to accept a previously-received offer to establish a
  /// connection wit the remote peer. Once the answer message is ready, the
  /// LocalSdpReadytoSendCallback callback is invoked to deliver the message.
  virtual bool MRS_API CreateAnswer() noexcept = 0;

  /// Close the peer connection. After the connection is closed, it cannot be
  /// opened again with the same C++ object. Instantiate a new |PeerConnection|
  /// object instead to create a new connection. No-op if already closed.
  virtual void MRS_API Close() noexcept = 0;

  /// Check if the connection is closed. This returns |true| once |Close()| has
  /// been called.
  virtual bool MRS_API IsClosed() const noexcept = 0;

  //
  // Remote tracks
  //

  /// Callback fired when a remote track is added to the peer connection.
  using TrackAddedCallback = Callback<TrackKind>;

  /// Register a custom TrackAddedCallback.
  virtual void RegisterTrackAddedCallback(
      TrackAddedCallback&& callback) noexcept = 0;

  /// Callback fired when a remote track is removed from the peer connection.
  using TrackRemovedCallback = Callback<TrackKind>;

  /// Register a custom TrackRemovedCallback.
  virtual void RegisterTrackRemovedCallback(
      TrackRemovedCallback&& callback) noexcept = 0;

  //
  // Video
  //

  /// Register a custom callback invoked when a remote video frame has been
  /// received and decompressed, and is ready to be displayed locally.
  virtual void RegisterRemoteVideoFrameCallback(
      I420AFrameReadyCallback callback) noexcept = 0;

  /// Register a custom callback invoked when a remote video frame has been
  /// received and decompressed, and is ready to be displayed locally.
  virtual void RegisterRemoteVideoFrameCallback(
      Argb32FrameReadyCallback callback) noexcept = 0;

  /// Add a video track to the peer connection. If no RTP sender/transceiver
  /// exist, create a new one for that track.
  virtual MRS_API ErrorOr<RefPtr<LocalVideoTrack>> AddLocalVideoTrack(
      rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track) noexcept = 0;

  /// Remove a local video track from the peer connection.
  /// The underlying RTP sender/transceiver are kept alive but inactive.
  virtual MRS_API webrtc::RTCError RemoveLocalVideoTrack(
      LocalVideoTrack& video_track) noexcept = 0;

  /// Remove all tracks sharing the given video track source.
  /// Note that currently video source sharing is not supported, so this will
  /// remove at most a single track backed by the given source.
  virtual MRS_API void RemoveLocalVideoTracksFromSource(
      ExternalVideoTrackSource& source) noexcept = 0;

  /// Rounding mode of video frame height for |SetFrameHeightRoundMode()|.
  /// This is only used on HoloLens 1 (UWP x86).
  enum class FrameHeightRoundMode {
    /// Leave frames unchanged.
    kNone = 0,

    /// Crop frame height to the nearest multiple of 16.
    /// ((height - nearestLowerMultipleOf16) / 2) rows are cropped from the top
    /// and (height - nearestLowerMultipleOf16 - croppedRowsTop) rows are
    /// cropped from the bottom.
    kCrop = 1,

    /// Pad frame height to the nearest multiple of 16.
    /// ((nearestHigherMultipleOf16 - height) / 2) rows are added symmetrically
    /// at the top and (nearestHigherMultipleOf16 - height - addedRowsTop) rows
    /// are added symmetrically at the bottom.
    kPad = 2
  };

  /// [HoloLens 1 only]
  /// Use this function to select whether resolutions where height is not
  /// multiple of 16 should be cropped, padded or left unchanged. Defaults to
  /// FrameHeightRoundMode::kCrop to avoid severe artifacts produced by the
  /// H.264 hardware encoder. The default value is applied when creating the
  /// first peer connection, so can be overridden after it.
  static void SetFrameHeightRoundMode(FrameHeightRoundMode value);

  //
  // Audio
  //

  /// Register a custom callback invoked when a local audio frame is ready to be
  /// output.
  ///
  /// FIXME - Current implementation of AddSink() for the local audio capture
  /// device is no-op. So this callback is never fired.
  virtual void RegisterLocalAudioFrameCallback(
      AudioFrameReadyCallback callback) noexcept = 0;

  /// Register a custom callback invoked when a remote audio frame has been
  /// received and uncompressed, and is ready to be output locally.
  virtual void RegisterRemoteAudioFrameCallback(
      AudioFrameReadyCallback callback) noexcept = 0;

  /// Add to the peer connection an audio track backed by a local audio capture
  /// device. If no RTP sender/transceiver exist, create a new one for that
  /// track.
  ///
  /// Note: currently a single local video track is supported per peer
  /// connection.
  virtual bool MRS_API AddLocalAudioTrack(
      rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track) noexcept = 0;

  /// Remove the existing local audio track from the peer connection.
  /// The underlying RTP sender/transceiver are kept alive but inactive.
  ///
  /// Note: currently a single local audio track is supported per peer
  /// connection.
  virtual void MRS_API RemoveLocalAudioTrack() noexcept = 0;

  /// Enable or disable the local audio track. Disabled audio tracks are still
  /// active but are silent, and do not consume network bandwidth. Additionally,
  /// enabling/disabling the local audio track does not require an SDP exchange.
  /// Therefore this is a cheaper alternative to removing and re-adding the
  /// track.
  ///
  /// Note: currently a single local audio track is supported per peer
  /// connection.
  virtual void MRS_API
  SetLocalAudioTrackEnabled(bool enabled = true) noexcept = 0;

  /// Check if the local audio frame is enabled.
  ///
  /// Note: currently a single local audio track is supported per peer
  /// connection.
  virtual bool MRS_API IsLocalAudioTrackEnabled() const noexcept = 0;

  //
  // Data channel
  //

  /// Callback invoked by the native layer when a new data channel is received
  /// from the remote peer and added locally.
  using DataChannelAddedCallback =
      Callback<mrsDataChannelInteropHandle, DataChannelHandle>;

  /// Callback invoked by the native layer when a data channel is removed from
  /// the remote peer and removed locally.
  using DataChannelRemovedCallback =
      Callback<mrsDataChannelInteropHandle, DataChannelHandle>;

  /// Register a custom callback invoked when a new data channel is received
  /// from the remote peer and added locally.
  virtual void RegisterDataChannelAddedCallback(
      DataChannelAddedCallback callback) noexcept = 0;

  /// Register a custom callback invoked when a data channel is removed by the
  /// remote peer and removed locally.
  virtual void RegisterDataChannelRemovedCallback(
      DataChannelRemovedCallback callback) noexcept = 0;

  /// Create a new data channel and add it to the peer connection.
  /// This invokes the DataChannelAdded callback.
  ErrorOr<std::shared_ptr<DataChannel>> MRS_API virtual AddDataChannel(
      int id,
      std::string_view label,
      bool ordered,
      bool reliable,
      mrsDataChannelInteropHandle dataChannelInteropHandle) noexcept = 0;

  /// Close and remove a given data channel.
  /// This invokes the DataChannelRemoved callback.
  virtual void MRS_API
  RemoveDataChannel(const DataChannel& data_channel) noexcept = 0;

  /// Close and remove all data channels at once.
  /// This invokes the DataChannelRemoved callback for each data channel.
  virtual void MRS_API RemoveAllDataChannels() noexcept = 0;

  /// Notification from a non-negotiated DataChannel that it is open, so that
  /// the PeerConnection can fire a DataChannelAdded event. This is called
  /// automatically by non-negotiated data channels; do not call manually.
  virtual void MRS_API
  OnDataChannelAdded(const DataChannel& data_channel) noexcept = 0;

  // TODO
  void GetStats(webrtc::RTCStatsCollectorCallback* callback);

  //
  // Advanced use
  //

  virtual mrsResult RegisterInteropCallbacks(
      const mrsPeerConnectionInteropCallbacks& callbacks) noexcept = 0;
};

}  // namespace Microsoft::MixedReality::WebRTC
