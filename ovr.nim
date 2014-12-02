#****************************************************************************
# Copyright   :   Copyright 2014 Oculus VR, LLC All Rights reserved.
#
# Licensed under the Oculus VR Rift SDK License Version 3.2 (the "License");
# you may not use the Oculus VR Rift SDK except in compliance with the License,
# which is provided at the time of installation or download, or which
# otherwise accompanies this software in either electronic or hard copy form.
#
# You may obtain a copy of the License at
#
# http://www.oculusvr.com/licenses/LICENSE-3.2
#
# Unless required by applicable law or agreed to in writing, the Oculus VR SDK
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#***************************************************************************
# Exposes all general Rift functionality.
# Please see the Oculus Developer Guide for detailed information about using the SDK in your native applications.
#***************************************************************************

{.deadCodeElim: on.}
when defined(windows):
    const libovr* = "ovr.dll"
elif defined(macosx):
    const libovr* = "libovr.dylib"
else:
    const libovr* = "libovr.so.0.1.1"

type
  Bool* = char

##define ENABLE_LATENCY_TESTER

#----------------------------------------------------------------------------
# ***** Simple Math Structures
# A 2D vector with integer components.
type
  Vector2i* = object
    x*: cint
    y*: cint


# A 2D size with integer components.
type
  Sizei* = object
    w*: cint
    h*: cint


# A 2D rectangle with a position and size.
# All components are integers.
type
  Recti* = object
    Pos*: Vector2i
    Size*: Sizei


# A quaternion rotation.
type
  Quatf* = object
    x*: cfloat
    y*: cfloat
    z*: cfloat
    w*: cfloat


# A 2D vector with float components.
type
  Vector2f* = object
    x*: cfloat
    y*: cfloat


# A 3D vector with float components.
type
  Vector3f* = object
    x*: cfloat
    y*: cfloat
    z*: cfloat


# A 4x4 matrix with float elements.
type
  Matrix4f* = object
    M*: array[4, array[4, cfloat]]


# Position and orientation together.
type
  Posef* = object
    Orientation*: Quatf
    Position*: Vector3f


# A full pose (rigid body) configuration with first and second derivatives.
type
  PoseStatef* = object
    ThePose*: Posef
    AngularVelocity*: Vector3f
    LinearVelocity*: Vector3f
    AngularAcceleration*: Vector3f
    LinearAcceleration*: Vector3f
    TimeInSeconds*: cdouble   # Absolute time of this state sample.
 

# Field Of View (FOV) in tangent of the angle units.
# As an example, for a standard 90 degree vertical FOV, we would
# have: { UpTan = tan(90 degrees / 2), DownTan = tan(90 degrees / 2) }.
type
  FovPort* = object
    UpTan*: cfloat            # The tangent of the angle between the viewing vector and the top edge of the field of view.
    # The tangent of the angle between the viewing vector and the bottom edge of the field of view.
    DownTan*: cfloat          # The tangent of the angle between the viewing vector and the left edge of the field of view.
    LeftTan*: cfloat          # The tangent of the angle between the viewing vector and the right edge of the field of view.
    RightTan*: cfloat


#-----------------------------------------------------------------------------
# ***** HMD Types
# Enumerates all HMD types that we support.
type
  HmdType* {.size: sizeof(cint).} = enum
    Hmd_None = 0,
    Hmd_DK1 = 3,
    Hmd_DKHD = 4,
    Hmd_DK2 = 6,
    Hmd_Other # Some HMD other then the one in the enumeration.


# HMD capability bits reported by device.
type
  HmdCaps* {.size: sizeof(cint).} = enum
    # Read-only flags
    HmdCap_Present = 0x00000001, # The HMD is plugged in and detected by the system.
    HmdCap_Available = 0x00000002, # The HMD and its sensor are available
                                   # for ownership use, i.e. it is not
                                   # already owned by another application.
    HmdCap_Captured = 0x00000004, # Set to 'true' if we captured ownership
                                  # of this HMD.

    # These flags are intended for use with the new driver display mode.
    HmdCap_ExtendDesktop = 0x00000008, # (read only) Means the display driver
                                       # is in compatibility mode.

    # Modifiable flags (through ovrHmd_SetEnabledCaps).
    HmdCap_DisplayOff = 0x00000040, # Turns off HMD screen and output
                                    # (only if 'ExtendDesktop' is off).
    HmdCap_LowPersistence = 0x00000080, # HMD supports low persistence mode.
    HmdCap_DynamicPrediction = 0x00000200, # Adjust prediction dynamically
                                           # based on internally measured latency.
    HmdCap_NoVSync = 0x00001000, # Support rendering without VSync for debugging.

    HmdCap_NoMirrorToWindow = 0x00002000, # Disables mirroring of HMD output
                                          # to the window. This may improve
                                          # rendering performance slightly
                                          # (only if 'ExtendDesktop' is off).

    # These flags are currently passed into the service. May change without notice.
    HmdCap_Service_Mask = 0x000023F0,

    # These bits can be modified by ovrHmd_SetEnabledCaps.
    HmdCap_Writable_Mask = 0x000033F0


# Tracking capability bits reported by the device.
# Used with ovrHmd_ConfigureTracking.
type
  TrackingCaps* {.size: sizeof(cint).} = enum
    TrackingCap_Orientation = 0x00000010, # Supports orientation tracking (IMU).
    TrackingCap_MagYawCorrection = 0x00000020, # Supports yaw drift correction via a magnetometer or other means.
    TrackingCap_Position = 0x00000040, # Supports positional tracking.
                                       # Overrides the other flags. Indicates that the application
                                       # doesn't care about tracking settings. This is the internal
                                       # default before ovrHmd_ConfigureTracking is called.
    TrackingCap_Idle = 0x00000100


# Distortion capability bits reported by device.
# Used with ovrHmd_ConfigureRendering and ovrHmd_CreateDistortionMesh.
type
  DistortionCaps* {.size: sizeof(cint).} = enum
    DistortionCap_Chromatic = 0x00000001, # Supports chromatic aberration correction.
    DistortionCap_TimeWarp = 0x00000002, # Supports timewarp.
    DistortionCap_Vignette = 0x00000008, # Supports vignetting around the edges of the view.
    DistortionCap_NoRestore = 0x00000010, # Do not save and restore the graphics state when rendering distortion.
    DistortionCap_FlipInput = 0x00000020, # Flip the vertical texture coordinate of input images.
    DistortionCap_SRGB = 0x00000040, # Assume input images are in sRGB gamma-corrected color space.
    DistortionCap_Overdrive = 0x00000080, # Overdrive brightness transitions to reduce artifacts on DK2+ displays
    DistortionCap_HqDistortion = 0x00000100, # High-quality sampling of distortion buffer for anti-aliasing
    DistortionCap_LinuxDevFullscreen = 0x00000200, # Indicates window is fullscreen on a device when set. The SDK will automatically apply distortion mesh rotation if needed.
    DistortionCap_ProfileNoTimewarpSpinWaits = 0x00010000 # Use when profiling with timewarp to remove false positives


# Specifies which eye is being used for rendering.
# This type explicitly does not include a third "NoStereo" option, as such is
# not required for an HMD-centered API.
type
  EyeType* {.size: sizeof(cint).} = enum
    Eye_Left = 0, Eye_Right = 1, Eye_Count = 2


# This is a complete descriptor of the HMD.
type
  HmdDesc* = object
    Handle*: pointer    # Internal handle of this HMD.
    # This HMD's type.
    Type*: HmdType            # Name string describing the product: "Oculus Rift DK1", etc.
    ProductName*: cstring
    Manufacturer*: cstring    # HID Vendor and ProductId of the device.
    VendorId*: cshort
    ProductId*: cshort        # Sensor (and display) serial number.
    SerialNumber*: array[24, char] # Sensor firmware version.
    FirmwareMajor*: cshort
    FirmwareMinor*: cshort    # External tracking camera frustum dimensions (if present).
    CameraFrustumHFovInRadians*: cfloat
    CameraFrustumVFovInRadians*: cfloat
    CameraFrustumNearZInMeters*: cfloat
    CameraFrustumFarZInMeters*: cfloat # Capability bits described by ovrHmdCaps.
    HmdCaps*: cuint           # Capability bits described by ovrTrackingCaps.
    TrackingCaps*: cuint      # Capability bits described by ovrDistortionCaps.
    DistortionCaps*: cuint    # These define the recommended and maximum optical FOVs for the HMD.
    DefaultEyeFov*: array[Eye_Count, FovPort]
    MaxEyeFov*: array[Eye_Count, FovPort] # Preferred eye rendering order for best performance.
                                          # Can help reduce latency on sideways-scanned screens.
    EyeRenderOrder*: array[Eye_Count, EyeType] # Resolution of the full HMD screen (both eyes) in pixels.
    Resolution*: Sizei        # Location of the application window on the desktop (or 0,0).
    WindowsPos*: Vector2i # Display that the HMD should present on.
                          # TBD: It may be good to remove this information relying on WindowPos instead.
                          # Ultimately, we may need to come up with a more convenient alternative,
                          # such as API-specific functions that return adapter, or something that will
                          # work with our monitor driver.
                          # Windows: (e.g. "\\\\.\\DISPLAY3", can be used in EnumDisplaySettings/CreateDC).
    DisplayDeviceName*: cstring # MacOS:
    DisplayId*: cint


# Simple type ovrHmd is used in ovrHmd_* calls.
type
  Hmd* = ptr HmdDesc

# Bit flags describing the current status of sensor tracking.
type
  StatusBits* {.size: sizeof(cint).} = enum
    Status_OrientationTracked = 0x00000001, # Orientation is currently tracked (connected and in use).
    Status_PositionTracked = 0x00000002, # Position is currently tracked (false if out of range).
    Status_CameraPoseTracked = 0x00000004, # Camera pose is currently tracked.
    Status_PositionConnected = 0x00000020, # Position tracking hardware is connected.
    Status_HmdConnected = 0x00000080


# Specifies a reading we can query from the sensor.
type
  SensorData* = object
    Accelerometer*: Vector3f  # Acceleration reading in m/s^2.
    Gyro*: Vector3f           # Rotation rate in rad/s.
    Magnetometer*: Vector3f   # Magnetic field in Gauss.
    Temperature*: cfloat      # Temperature of the sensor in degrees Celsius.
    TimeInSeconds*: cfloat    # Time when the reported IMU reading took place, in seconds.
 

# Tracking state at a given absolute time (describes predicted HMD pose etc).
# Returned by ovrHmd_GetTrackingState.
type
  TrackingState* = object
    HeadPose*: PoseStatef # Predicted head pose (and derivatives) at the requested absolute time.
                          # The look-ahead interval is equal to (HeadPose.TimeInSeconds - RawSensorData.TimeInSeconds).
    # Current pose of the external camera (if present).
    # This pose includes camera tilt (roll and pitch). For a leveled coordinate
    # system use LeveledCameraPose.
    CameraPose*: Posef # Camera frame aligned with gravity.
                       # This value includes position and yaw of the camera, but not roll and pitch.
                       # It can be used as a reference point to render real-world objects in the correct location.
    LeveledCameraPose*: Posef # The most recent sensor data received from the HMD.
    RawSensorData*: SensorData # Tracking status described by ovrStatusBits.
    StatusFlags*: cuint       #/ 0.4.1
                              # Measures the time from receiving the camera frame until vision CPU processing completes.
    LastVisionProcessingTime*: cdouble #/ 0.4.3
                                       # Measures the time from exposure until the pose is available for the frame, including processing time.
    LastVisionFrameLatency*: cdouble # Tag the vision processing results to a certain frame counter number.
    LastCameraFrameCounter*: uint32


# Frame timing data reported by ovrHmd_BeginFrameTiming() or ovrHmd_BeginFrame().
type
  FrameTiming* = object
    DeltaSeconds*: cfloat # The amount of time that has passed since the previous frame's
                          # ThisFrameSeconds value (usable for movement scaling).
                          # This will be clamped to no more than 0.1 seconds to prevent
                          # excessive movement after pauses due to loading or initialization.
    # It is generally expected that the following holds:
    # ThisFrameSeconds < TimewarpPointSeconds < NextFrameSeconds <
    # EyeScanoutSeconds[EyeOrder[0]] <= ScanoutMidpointSeconds <= EyeScanoutSeconds[EyeOrder[1]].
    # Absolute time value when rendering of this frame began or is expected to
    # begin. Generally equal to NextFrameSeconds of the previous frame. Can be used
    # for animation timing.
    ThisFrameSeconds*: cdouble # Absolute point when IMU expects to be sampled for this frame.
    TimewarpPointSeconds*: cdouble # Absolute time when frame Present followed by GPU Flush will finish and the next frame begins.
    NextFrameSeconds*: cdouble # Time when half of the screen will be scanned out. Can be passed as an absolute time
                               # to ovrHmd_GetTrackingState() to get the predicted general orientation.
    ScanoutMidpointSeconds*: cdouble # Timing points when each eye will be scanned out to display. Used when rendering each eye.
    EyeScanoutSeconds*: array[2, cdouble]


# Rendering information for each eye. Computed by either ovrHmd_ConfigureRendering()
# or ovrHmd_GetRenderDesc() based on the specified FOV. Note that the rendering viewport
# is not included here as it can be specified separately and modified per frame through:
#    (a) ovrHmd_GetRenderScaleAndOffset in the case of client rendered distortion,
# or (b) passing different values via ovrTexture in the case of SDK rendered distortion.
type
  EyeRenderDesc* = object
    Eye*: EyeType
    Fov*: FovPort
    DistortedViewport*: Recti # Distortion viewport.
    PixelsPerTanAngleAtCenter*: Vector2f # How many display pixels will fit in tan(angle) = 1.
    HmdToEyeViewOffset*: Vector3f # Translation to be applied to view matrix for each eye offset.
 

#-----------------------------------------------------------------------------
# ***** Platform-independent Rendering Configuration
# These types are used to hide platform-specific details when passing
# render device, OS, and texture data to the API.
#
# The benefit of having these wrappers versus platform-specific API functions is
# that they allow game glue code to be portable. A typical example is an
# engine that has multiple back ends, say GL and D3D. Portable code that calls
# these back ends may also use LibOVR. To do this, back ends can be modified
# to return portable types such as ovrTexture and ovrRenderAPIConfig.
type
  RenderAPIType* {.size: sizeof(cint).} = enum
    RenderAPI_None, RenderAPI_OpenGL, RenderAPI_Android_GLES, # May include extra native window pointers, etc.
    RenderAPI_D3D9, RenderAPI_D3D10, RenderAPI_D3D11, RenderAPI_Count


# Platform-independent part of rendering API-configuration data.
# It is a part of ovrRenderAPIConfig, passed to ovrHmd_Configure.
type
  RenderAPIConfigHeader* = object
    API*: RenderAPIType
    RTSize*: Sizei
    Multisample*: cint


# Contains platform-specific information for rendering.
type
  RenderAPIConfig* = object
    Header*: RenderAPIConfigHeader
    PlatformData*: array[8, ptr cuint]


# Platform-independent part of the eye texture descriptor.
# It is a part of ovrTexture, passed to ovrHmd_EndFrame.
# If RenderViewport is all zeros then the full texture will be used.
type
  TextureHeader* = object
    API*: RenderAPIType
    TextureSize*: Sizei
    RenderViewport*: Recti    # Pixel viewport in texture that holds eye image.
 

# Contains platform-specific information about a texture.
type
  Texture* = object
    Header*: TextureHeader
    PlatformData*: array[8, ptr cuint]


#
# ----------------------------------------------------------------------------
# ***** API Interfaces
# Basic steps to use the API:
#
# Setup:
#  * ovrInitialize()
#  * ovrHMD hmd = ovrHmd_Create(0)
#  * Use hmd members and ovrHmd_GetFovTextureSize() to determine graphics configuration.
#  * Call ovrHmd_ConfigureTracking() to configure and initialize tracking.
#  * Call ovrHmd_ConfigureRendering() to setup graphics for SDK rendering,
#    which is the preferred approach.
#    Please refer to "Client Distorton Rendering" below if you prefer to do that instead.
#  * If the ovrHmdCap_ExtendDesktop flag is not set, then use ovrHmd_AttachToWindow to
#    associate the relevant application window with the hmd.
#  * Allocate render target textures as needed.
#
# Game Loop:
#  * Call ovrHmd_BeginFrame() to get the current frame timing information.
#  * Render each eye using ovrHmd_GetEyePoses or ovrHmd_GetHmdPosePerEye to get
#    the predicted hmd pose and each eye pose.
#  * Call ovrHmd_EndFrame() to render the distorted textures to the back buffer
#    and present them on the hmd.
#
# Shutdown:
#  * ovrHmd_Destroy(hmd)
#  * ovr_Shutdown()
#

# ovr_InitializeRenderingShim initializes the rendering shim appart from everything
# else in LibOVR. This may be helpful if the application prefers to avoid
# creating any OVR resources (allocations, service connections, etc) at this point.
# ovr_InitializeRenderingShim does not bring up anything within LibOVR except the
# necessary hooks to enable the Direct-to-Rift functionality.
#
# Either ovr_InitializeRenderingShim() or ovr_Initialize() must be called before any
# Direct3D or OpenGL initilization is done by applictaion (creation of devices, etc).
# ovr_Initialize() must still be called after to use the rest of LibOVR APIs.
proc initializeRenderingShim*() {.cdecl,
                                   importc: "ovr_InitializeRenderingShim",
                                   dynlib: libovr.}

# Library init/shutdown, must be called around all other OVR code.
# No other functions calls besides ovr_InitializeRenderingShim are allowed
# before ovr_Initialize succeeds or after ovr_Shutdown.
# Initializes all Oculus functionality.
proc initialize*(): Bool {.cdecl, importc: "ovr_Initialize", dynlib: libovr.}

# Shuts down all Oculus functionality.
proc shutdown*() {.cdecl, importc: "ovr_Shutdown", dynlib: libovr.}

# Returns version string representing libOVR version. Static, so
# string remains valid for app lifespan
proc getVersionString*(): cstring {.cdecl, importc: "ovr_GetVersionString",
                                     dynlib: libovr.}


# Detects or re-detects HMDs and reports the total number detected.
# Users can get information about each HMD by calling ovrHmd_Create with an index.
proc detectHMD*(): cint {.cdecl, importc: "ovrHmd_Detect", dynlib: libovr.}

# Creates a handle to an HMD which doubles as a description structure.
# Index can [0 .. ovrHmd_Detect()-1]. Index mappings can cange after each ovrHmd_Detect call.
# If not null, then the returned handle must be freed with ovrHmd_Destroy.
proc createHMD*(index: cint): Hmd {.cdecl, importc: "ovrHmd_Create",
                                     dynlib: libovr.}
proc destroy*(hmd: Hmd) {.cdecl, importc: "ovrHmd_Destroy", dynlib: libovr.}

# Creates a 'fake' HMD used for debugging only. This is not tied to specific hardware,
# but may be used to debug some of the related rendering.
proc createDebugHMD*(typ: HmdType): Hmd {.cdecl,
    importc: "ovrHmd_CreateDebug", dynlib: libovr.}

# Returns last error for HMD state. Returns null for no error.
# String is valid until next call or GetLastError or HMD is destroyed.
# Pass null hmd to get global errors (during create etc).
proc getLastError*(hmd: Hmd): cstring {.cdecl,
    importc: "ovrHmd_GetLastError", dynlib: libovr.}

# Platform specific function to specify the application window whose output will be
# displayed on the HMD. Only used if the ovrHmdCap_ExtendDesktop flag is false.
#   Windows: SwapChain associated with this window will be displayed on the HMD.
#            Specify 'destMirrorRect' in window coordinates to indicate an area
#            of the render target output that will be mirrored from 'sourceRenderTargetRect'.
#            Null pointers mean "full size".
# @note Source and dest mirror rects are not yet implemented.
proc attachToWindow*(hmd: Hmd; window: pointer; destMirrorRect: ptr Recti;
                         sourceRenderTargetRect: ptr Recti): Bool {.cdecl,
    importc: "ovrHmd_AttachToWindow", dynlib: libovr.}

# Returns capability bits that are enabled at this time as described by ovrHmdCaps.
# Note that this value is different font ovrHmdDesc::HmdCaps, which describes what
# capabilities are available for that HMD.
proc getEnabledCaps*(hmd: Hmd): cuint {.cdecl,
    importc: "ovrHmd_GetEnabledCaps", dynlib: libovr.}

# Modifies capability bits described by ovrHmdCaps that can be modified,
# such as ovrHmdCap_LowPersistance.
proc setEnabledCaps*(hmd: Hmd; hmdCaps: cuint) {.cdecl,
    importc: "ovrHmd_SetEnabledCaps", dynlib: libovr.}

#-----------------------------------------------------------------------------
# ***** Tracking Interface
# All tracking interface functions are thread-safe, allowing tracking state to be sampled
# from different threads.
# ConfigureTracking starts sensor sampling, enabling specified capabilities,
#    described by ovrTrackingCaps.
#  - supportedTrackingCaps specifies support that is requested. The function will succeed
#   even if these caps are not available (i.e. sensor or camera is unplugged). Support
#    will automatically be enabled if such device is plugged in later. Software should
#    check ovrTrackingState.StatusFlags for real-time status.
#  - requiredTrackingCaps specify sensor capabilities required at the time of the call.
#    If they are not available, the function will fail. Pass 0 if only specifying
#    supportedTrackingCaps.
#  - Pass 0 for both supportedTrackingCaps and requiredTrackingCaps to disable tracking.
proc configureTracking*(hmd: Hmd; supportedTrackingCaps: cuint;
                            requiredTrackingCaps: cuint): Bool {.cdecl,
    importc: "ovrHmd_ConfigureTracking", dynlib: libovr.}

# Re-centers the sensor orientation.
# Normally this will recenter the (x,y,z) translational components and the yaw
# component of orientation.
proc recenterPose*(hmd: Hmd) {.cdecl, importc: "ovrHmd_RecenterPose",
                                   dynlib: libovr.}
# Returns tracking state reading based on the specified absolute system time.
# Pass an absTime value of 0.0 to request the most recent sensor reading. In this case
# both PredictedPose and SamplePose will have the same value.
# ovrHmd_GetEyePoses relies on this function internally.
# This may also be used for more refined timing of FrontBuffer rendering logic, etc.
proc getTrackingState*(hmd: Hmd; absTime: cdouble): TrackingState {.cdecl,
    importc: "ovrHmd_GetTrackingState", dynlib: libovr.}

#-----------------------------------------------------------------------------
# ***** Graphics Setup
# Calculates the recommended texture size for rendering a given eye within the HMD
# with a given FOV cone. Higher FOV will generally require larger textures to
# maintain quality.
#  - pixelsPerDisplayPixel specifies the ratio of the number of render target pixels
#    to display pixels at the center of distortion. 1.0 is the default value. Lower
#    values can improve performance.
proc getFovTextureSize*(hmd: Hmd; eye: EyeType; fov: FovPort;
                            pixelsPerDisplayPixel: cfloat): Sizei {.cdecl,
    importc: "ovrHmd_GetFovTextureSize", dynlib: libovr.}


#-----------------------------------------------------------------------------
# *****  Rendering API Thread Safety
#  All of rendering functions including the configure and frame functions
# are *NOT thread safe*. It is ok to use ConfigureRendering on one thread and handle
#  frames on another thread, but explicit synchronization must be done since
#  functions that depend on configured state are not reentrant.
#
#  As an extra requirement, any of the following calls must be done on
#  the render thread, which is the same thread that calls ovrHmd_BeginFrame
#  or ovrHmd_BeginFrameTiming.
#    - ovrHmd_EndFrame
#    - ovrHmd_GetEyeTimewarpMatrices
#-----------------------------------------------------------------------------
# *****  SDK Distortion Rendering Functions
# These functions support rendering of distortion by the SDK through direct
# access to the underlying rendering API, such as D3D or GL.
# This is the recommended approach since it allows better support for future
# Oculus hardware, and enables a range of low-level optimizations.
# Configures rendering and fills in computed render parameters.
# This function can be called multiple times to change rendering settings.
# eyeRenderDescOut is a pointer to an array of two ovrEyeRenderDesc structs
# that are used to return complete rendering information for each eye.
#  - apiConfig provides D3D/OpenGL specific parameters. Pass null
#    to shutdown rendering and release all resources.
#  - distortionCaps describe desired distortion settings.
proc configureRendering*(hmd: Hmd; apiConfig: ptr RenderAPIConfig;
                             distortionCaps: cuint; eyeFovIn: array[2, FovPort];
                             eyeRenderDescOut: array[2, EyeRenderDesc]): Bool {.
    cdecl, importc: "ovrHmd_ConfigureRendering", dynlib: libovr.}

# Begins a frame, returning timing information.
# This should be called at the beginning of the game rendering loop (on the render thread).
# Pass 0 for the frame index if not using ovrHmd_GetFrameTiming.
proc beginFrame*(hmd: Hmd; frameIndex: cuint): FrameTiming {.cdecl,
    importc: "ovrHmd_BeginFrame", dynlib: libovr.}

# Ends a frame, submitting the rendered textures to the frame buffer.
# - RenderViewport within each eyeTexture can change per frame if necessary.
# - 'renderPose' will typically be the value returned from ovrHmd_GetEyePoses,
#   ovrHmd_GetHmdPosePerEye but can be different if a different head pose was
#   used for rendering.
# - This may perform distortion and scaling internally, assuming is it not
#   delegated to another thread.
# - Must be called on the same thread as BeginFrame.
# - *** This Function will call Present/SwapBuffers and potentially wait for GPU Sync ***.
proc endFrame*(hmd: Hmd; renderPose: array[2, Posef];
                   eyeTexture: array[2, Texture]) {.cdecl,
    importc: "ovrHmd_EndFrame", dynlib: libovr.}

# Returns predicted head pose in outHmdTrackingState and offset eye poses in outEyePoses
# as an atomic operation. Caller need not worry about applying HmdToEyeViewOffset to the
# returned outEyePoses variables.
# - Thread-safe function where caller should increment frameIndex with every frame
#   and pass the index where applicable to functions called on the  rendering thread.
# - hmdToEyeViewOffset[2] can be ovrEyeRenderDesc.HmdToEyeViewOffset returned from
#   ovrHmd_ConfigureRendering or ovrHmd_GetRenderDesc. For monoscopic rendering,
#   use a vector that is the average of the two vectors for both eyes.
# - If frameIndex is not being used, pass in 0.
# - Assuming outEyePoses are used for rendering, it should be passed into ovrHmd_EndFrame.
# - If called doesn't need outHmdTrackingState, it can be NULL
proc getEyePoses*(hmd: Hmd; frameIndex: cuint;
                      hmdToEyeViewOffset: array[2, Vector3f];
                      outEyePoses: array[2, Posef];
                      outHmdTrackingState: ptr TrackingState) {.cdecl,
    importc: "ovrHmd_GetEyePoses", dynlib: libovr.}

# DEPRECATED: Prefer using ovrHmd_GetEyePoses instead
# Function was previously called ovrHmd_GetEyePose
# Returns the predicted head pose to use when rendering the specified eye.
# - Important: Caller must apply HmdToEyeViewOffset before using ovrPosef for rendering
# - Must be called between ovrHmd_BeginFrameTiming and ovrHmd_EndFrameTiming.
# - If the pose is used for rendering the eye, it should be passed to ovrHmd_EndFrame.
# - Parameter 'eye' is used for prediction timing only
proc getHmdPosePerEye*(hmd: Hmd; eye: EyeType): Posef {.cdecl,
    importc: "ovrHmd_GetHmdPosePerEye", dynlib: libovr.}


#-----------------------------------------------------------------------------
# *****  Client Distortion Rendering Functions
# These functions provide the distortion data and render timing support necessary to allow
# client rendering of distortion. Client-side rendering involves the following steps:
#
#  1. Setup ovrEyeDesc based on the desired texture size and FOV.
#     Call ovrHmd_GetRenderDesc to get the necessary rendering parameters for each eye.
#
#  2. Use ovrHmd_CreateDistortionMesh to generate the distortion mesh.
#
#  3. Use ovrHmd_BeginFrameTiming, ovrHmd_GetEyePoses, and ovrHmd_BeginFrameTiming in
#     the rendering loop to obtain timing and predicted head orientation when rendering each eye.
#      - When using timewarp, use ovr_WaitTillTime after the rendering and gpu flush, followed
#        by ovrHmd_GetEyeTimewarpMatrices to obtain the timewarp matrices used
#        by the distortion pixel shader. This will minimize latency.
#
# Computes the distortion viewport, view adjust, and other rendering parameters for
# the specified eye. This can be used instead of ovrHmd_ConfigureRendering to do
# setup for client rendered distortion.
proc getRenderDesc*(hmd: Hmd; eyeType: EyeType; fov: FovPort): EyeRenderDesc {.
    cdecl, importc: "ovrHmd_GetRenderDesc", dynlib: libovr.}

# Describes a vertex used by the distortion mesh. This is intended to be converted into
# the engine-specific format. Some fields may be unused based on the ovrDistortionCaps
# flags selected. TexG and TexB, for example, are not used if chromatic correction is
# not requested.
type
  DistortionVertex* = object
    ScreenPosNDC*: Vector2f   # [-1,+1],[-1,+1] over the entire framebuffer.
    TimeWarpFactor*: cfloat   # Lerp factor between time-warp matrices.
                              # Can be encoded in Pos.z.
    VignetteFactor*: cfloat   # Vignette fade factor. Can be encoded in Pos.w.
    TanEyeAnglesR*: Vector2f
    TanEyeAnglesG*: Vector2f
    TanEyeAnglesB*: Vector2f


# Describes a full set of distortion mesh data, filled in
# by ovrHmd_CreateDistortionMesh.
# Contents of this data structure, if not null, should be freed
# by ovrHmd_DestroyDistortionMesh.
type
  DistortionMesh* = object
    pVertexData*: ptr DistortionVertex
    pIndexData*: ptr cushort
    VertexCount*: cuint
    IndexCount*: cuint


# Generate distortion mesh per eye.
# Distortion capabilities will depend on 'distortionCaps' flags. Users should
# render using the appropriate shaders based on their settings.
# Distortion mesh data will be allocated and written into the ovrDistortionMesh data structure,
# which should be explicitly freed with ovrHmd_DestroyDistortionMesh.
# Users should call ovrHmd_GetRenderScaleAndOffset to get uvScale and Offset values for rendering.
# The function shouldn't fail unless theres is a configuration or memory error, in which case
# ovrDistortionMesh values will be set to null.
# This is the only function in the SDK reliant on eye relief, currently imported from profiles,
# or overridden here.
proc createDistortionMesh*(hmd: Hmd; eyeType: EyeType; fov: FovPort;
                               distortionCaps: cuint;
                               meshData: ptr DistortionMesh): Bool {.cdecl,
    importc: "ovrHmd_CreateDistortionMesh", dynlib: libovr.}

# Used to free the distortion mesh allocated by ovrHmd_GenerateDistortionMesh. meshData elements
# are set to null and zeroes after the call.
proc destroyDistortionMesh*(meshData: ptr DistortionMesh) {.cdecl,
    importc: "ovrHmd_DestroyDistortionMesh", dynlib: libovr.}

# Computes updated 'uvScaleOffsetOut' to be used with a distortion if render target size or
# viewport changes after the fact. This can be used to adjust render size every frame if desired.
proc getRenderScaleAndOffset*(fov: FovPort; textureSize: Sizei;
                                  renderViewport: Recti;
                                  uvScaleOffsetOut: array[2, Vector2f]) {.cdecl,
    importc: "ovrHmd_GetRenderScaleAndOffset", dynlib: libovr.}

# Thread-safe timing function for the main thread. Caller should increment frameIndex
# with every frame and pass the index where applicable to functions called on the
# rendering thread.
proc getFrameTiming*(hmd: Hmd; frameIndex: cuint): FrameTiming {.cdecl,
    importc: "ovrHmd_GetFrameTiming", dynlib: libovr.}

# Called at the beginning of the frame on the rendering thread.
# Pass frameIndex == 0 if ovrHmd_GetFrameTiming isn't being used. Otherwise,
# pass the same frame index as was used for GetFrameTiming on the main thread.
proc beginFrameTiming*(hmd: Hmd; frameIndex: cuint): FrameTiming {.cdecl,
    importc: "ovrHmd_BeginFrameTiming", dynlib: libovr.}

# Marks the end of client distortion rendered frame, tracking the necessary timing information.
# This function must be called immediately after Present/SwapBuffers + GPU sync. GPU sync is
# important before this call to reduce latency and ensure proper timing.
proc endFrameTiming*(hmd: Hmd) {.cdecl, importc: "ovrHmd_EndFrameTiming",
                                     dynlib: libovr.}

# Initializes and resets frame time tracking. This is typically not necessary, but
# is helpful if game changes vsync state or video mode. vsync is assumed to be on if this
# isn't called. Resets internal frame index to the specified number.
proc resetFrameTiming*(hmd: Hmd; frameIndex: cuint) {.cdecl,
    importc: "ovrHmd_ResetFrameTiming", dynlib: libovr.}

# Computes timewarp matrices used by distortion mesh shader, these are used to adjust
# for head orientation change since the last call to ovrHmd_GetEyePoses
# when rendering this eye. The ovrDistortionVertex::TimeWarpFactor is used to blend between the
# matrices, usually representing two different sides of the screen.
# Must be called on the same thread as ovrHmd_BeginFrameTiming.
proc getEyeTimewarpMatrices*(hmd: Hmd; eye: EyeType; renderPose: Posef;
                                 twmOut: array[2, Matrix4f]) {.cdecl,
    importc: "ovrHmd_GetEyeTimewarpMatrices", dynlib: libovr.}


#-----------------------------------------------------------------------------
# ***** Stateless math setup functions
# Used to generate projection from ovrEyeDesc::Fov.
proc Matrix4f_Projection*(fov: FovPort; znear: cfloat; zfar: cfloat;
                          rightHanded: Bool): Matrix4f {.cdecl,
    importc: "ovrMatrix4f_Projection", dynlib: libovr.}

# Used for 2D rendering, Y is down
# orthoScale = 1.0f / pixelsPerTanAngleAtCenter
# orthoDistance = distance from camera, such as 0.8m
proc Matrix4f_OrthoSubProjection*(projection: Matrix4f; orthoScale: Vector2f;
                                  orthoDistance: cfloat;
                                  hmdToEyeViewOffsetX: cfloat): Matrix4f {.
    cdecl, importc: "ovrMatrix4f_OrthoSubProjection", dynlib: libovr.}

# Returns global, absolute high-resolution time in seconds. This is the same
# value as used in sensor messages.
proc getTimeInSeconds*(): cdouble {.cdecl, importc: "ovr_GetTimeInSeconds",
                                     dynlib: libovr.}
# Waits until the specified absolute time.
proc waitTillTime*(absTime: cdouble): cdouble {.cdecl,
    importc: "ovr_WaitTillTime", dynlib: libovr.}


# ----------------------------------------------------------------------------
# ***** Latency Test interface
# Does latency test processing and returns 'TRUE' if specified rgb color should
# be used to clear the screen.
proc processLatencyTest*(hmd: Hmd; rgbColorOut: array[3, cuchar]): Bool {.
    cdecl, importc: "ovrHmd_ProcessLatencyTest", dynlib: libovr.}

# Returns non-null string once with latency test result, when it is available.
# Buffer is valid until next call.
proc getLatencyTestResult*(hmd: Hmd): cstring {.cdecl,
    importc: "ovrHmd_GetLatencyTestResult", dynlib: libovr.}

# Returns the latency testing color in rgbColorOut to render when using a DK2
# Returns false if this feature is disabled or not-applicable (e.g. using a DK1)
proc getLatencyTest2DrawColor*(hmddesc: Hmd; rgbColorOut: array[3, cuchar]): Bool {.
    cdecl, importc: "ovrHmd_GetLatencyTest2DrawColor", dynlib: libovr.}


#-----------------------------------------------------------------------------
# ***** Health and Safety Warning Display interface
#
# Used by ovrhmd_GetHSWDisplayState to report the current display state.
type
  HSWDisplayState* = object
    Displayed*: Bool # If true then the warning should be currently visible
                     # and the following variables have meaning. Else there is no
                     # warning being displayed for this application on the given HMD.
    StartTime*: cdouble       # Absolute time when the warning was first displayed. See ovr_GetTimeInSeconds().
    DismissibleTime*: cdouble # Earliest absolute time when the warning can be dismissed. May be a time in the past.
 

# Returns the current state of the HSW display. If the application is doing the rendering of
# the HSW display then this function serves to indicate that the warning should be
# currently displayed. If the application is using SDK-based eye rendering then the SDK by
# default automatically handles the drawing of the HSW display. An application that uses
# application-based eye rendering should use this function to know when to start drawing the
# HSW display itself and can optionally use it in conjunction with ovrhmd_DismissHSWDisplay
# as described below.
#
# Example usage for application-based rendering:
#    bool HSWDisplayCurrentlyDisplayed = false; // global or class member variable
#    ovrHSWDisplayState hswDisplayState;
#    ovrhmd_GetHSWDisplayState(Hmd, &hswDisplayState);
#
#    if (hswDisplayState.Displayed && !HSWDisplayCurrentlyDisplayed) {
#        <insert model into the scene that stays in front of the user>
#        HSWDisplayCurrentlyDisplayed = true;
#    }
proc getHSWDisplayState*(hmd: Hmd; hasWarningState: ptr HSWDisplayState) {.
    cdecl, importc: "ovrHmd_GetHSWDisplayState", dynlib: libovr.}

# Dismisses the HSW display if the warning is dismissible and the earliest dismissal time
# has occurred. Returns true if the display is valid and could be dismissed. The application
# should recognize that the HSW display is being displayed (via ovrhmd_GetHSWDisplayState)
# and if so then call this function when the appropriate user input to dismiss the warning
# occurs.
#
# Example usage :
#    void ProcessEvent(int key) {
#        if (key == escape) {
#            ovrHSWDisplayState hswDisplayState;
#            ovrhmd_GetHSWDisplayState(hmd, &hswDisplayState);
#
#            if (hswDisplayState.Displayed && ovrhmd_DismissHSWDisplay(hmd)) {
#                <remove model from the scene>
#                HSWDisplayCurrentlyDisplayed = false;
#            }
#        }
#    }
proc dismissHSWDisplay*(hmd: Hmd): Bool {.cdecl,
    importc: "ovrHmd_DismissHSWDisplay", dynlib: libovr.}

# Get boolean property. Returns first element if property is a boolean array.
# Returns defaultValue if property doesn't exist.
proc getBool*(hmd: Hmd; propertyName: cstring; defaultVal: Bool): Bool {.
    cdecl, importc: "ovrHmd_GetBool", dynlib: libovr.}

# Modify bool property; false if property doesn't exist or is readonly.
proc setBool*(hmd: Hmd; propertyName: cstring; value: Bool): Bool {.cdecl,
    importc: "ovrHmd_SetBool", dynlib: libovr.}

# Get integer property. Returns first element if property is an integer array.
# Returns defaultValue if property doesn't exist.
proc getInt*(hmd: Hmd; propertyName: cstring; defaultVal: cint): cint {.
    cdecl, importc: "ovrHmd_GetInt", dynlib: libovr.}

# Modify integer property; false if property doesn't exist or is readonly.
proc setInt*(hmd: Hmd; propertyName: cstring; value: cint): Bool {.cdecl,
    importc: "ovrHmd_SetInt", dynlib: libovr.}

# Get float property. Returns first element if property is a float array.
# Returns defaultValue if property doesn't exist.
proc getFloat*(hmd: Hmd; propertyName: cstring; defaultVal: cfloat): cfloat {.
    cdecl, importc: "ovrHmd_GetFloat", dynlib: libovr.}

# Modify float property; false if property doesn't exist or is readonly.
proc setFloat*(hmd: Hmd; propertyName: cstring; value: cfloat): Bool {.
    cdecl, importc: "ovrHmd_SetFloat", dynlib: libovr.}

# Get float[] property. Returns the number of elements filled in, 0 if property doesn't exist.
# Maximum of arraySize elements will be written.
proc getFloatArray*(hmd: Hmd; propertyName: cstring; values: ptr cfloat;
                        arraySize: cuint): cuint {.cdecl,
    importc: "ovrHmd_GetFloatArray", dynlib: libovr.}

# Modify float[] property; false if property doesn't exist or is readonly.
proc setFloatArray*(hmd: Hmd; propertyName: cstring; values: ptr cfloat;
                        arraySize: cuint): Bool {.cdecl,
    importc: "ovrHmd_SetFloatArray", dynlib: libovr.}

# Get string property. Returns first element if property is a string array.
# Returns defaultValue if property doesn't exist.
# String memory is guaranteed to exist until next call to GetString or GetStringArray, or HMD is destroyed.
proc getString*(hmd: Hmd; propertyName: cstring; defaultVal: cstring): cstring {.
    cdecl, importc: "ovrHmd_GetString", dynlib: libovr.}

# Set string property
proc setString*(hmddesc: Hmd; propertyName: cstring; value: cstring): Bool {.
    cdecl, importc: "ovrHmd_SetString", dynlib: libovr.}

#
# ----------------------------------------------------------------------------
# ***** Logging
# Start performance logging. guid is optional and if included is written with each file entry.
# If called while logging is already active with the same filename, only the guid will be updated
# If called while logging is already active with a different filename, ovrHmd_StopPerfLog() will be called, followed by ovrHmd_StartPerfLog()
proc startPerfLog*(hmd: Hmd; fileName: cstring; userData1: cstring): Bool {.
    cdecl, importc: "ovrHmd_StartPerfLog", dynlib: libovr.}

# Stop performance logging.
proc stopPerfLog*(hmd: Hmd): Bool {.cdecl, importc: "ovrHmd_StopPerfLog",
                                        dynlib: libovr.}


const 
  KEY_USER* = "User"
  KEY_NAME* = "Name"
  KEY_GENDER* = "Gender"
  KEY_PLAYER_HEIGHT* = "PlayerHeight"
  KEY_EYE_HEIGHT* = "EyeHeight"
  KEY_IPD* = "IPD"
  KEY_NECK_TO_EYE_DISTANCE* = "NeckEyeDistance"
  KEY_EYE_RELIEF_DIAL* = "EyeReliefDial"
  KEY_EYE_TO_NOSE_DISTANCE* = "EyeToNoseDist"
  KEY_MAX_EYE_TO_PLATE_DISTANCE* = "MaxEyeToPlateDist"
  KEY_EYE_CUP* = "EyeCup"
  KEY_CUSTOM_EYE_RENDER* = "CustomEyeRender"
  KEY_CAMERA_POSITION* = "CenteredFromWorld"

# Default measurements empirically determined at Oculus to make us happy
# The neck model numbers were derived as an average of the male and female averages from ANSUR-88
# NECK_TO_EYE_HORIZONTAL = H22 - H43 = INFRAORBITALE_BACK_OF_HEAD - TRAGION_BACK_OF_HEAD
# NECK_TO_EYE_VERTICAL = H21 - H15 = GONION_TOP_OF_HEAD - ECTOORBITALE_TOP_OF_HEAD
# These were determined to be the best in a small user study, clearly beating out the previous default values
const 
  DEFAULT_GENDER* = "Unknown"
  DEFAULT_PLAYER_HEIGHT* = 1.778
  DEFAULT_EYE_HEIGHT* = 1.675
  DEFAULT_IPD* = 0.064
  DEFAULT_NECK_TO_EYE_HORIZONTAL* = 0.0805
  DEFAULT_NECK_TO_EYE_VERTICAL* = 0.075
  DEFAULT_EYE_RELIEF_DIAL* = 3
  DEFAULT_CAMERA_POSITION* = (0, 0, 0, 1, 0, 0, 0)
