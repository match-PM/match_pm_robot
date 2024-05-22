/**********************************************************************
 * Copyright (c) 2022 SmarAct GmbH
 *
 * File name: SmarActControl.h
 *
 * THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
 * WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
 * BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
 * FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
 * THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
 * REMAINS WITH YOU.
 * IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
 * OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
 **********************************************************************/

#ifndef SMARACT_CTL_H
#define SMARACT_CTL_H

#include "SmarActControlConstants.h"
#include <stdint.h>
#include <stdlib.h>

// @constants ApiVersion

#define SA_CTL_VERSION_MAJOR 1
#define SA_CTL_VERSION_MINOR 3
#define SA_CTL_VERSION_UPDATE 37

// @endconstants ApiVersion

#if defined(_WIN32)
#define SA_CTL_PLATFORM_WINDOWS
#elif defined(__linux__)
#define SA_CTL_PLATFORM_LINUX
#else
#error "unsupported platform"
#endif

#if defined(SA_CTL_PLATFORM_WINDOWS)
#if !defined(_SA_CTL_DIRECTLINK)
#ifdef SA_CTL_EXPORTS
#define SA_CTL_API __declspec(dllexport)
#else
#define SA_CTL_API __declspec(dllimport)
#endif
#else
#define SA_CTL_API
#endif
#define SA_CTL_CC __cdecl
#else
#define SA_CTL_API __attribute__((visibility("default")))
#define SA_CTL_CC
#endif

#ifdef __cplusplus
extern "C" {
#endif

/************
 * TYPEDEFS *
 ***********/

typedef uint32_t SA_CTL_DeviceHandle_t;
typedef uint32_t SA_CTL_TransmitHandle_t;
typedef uint32_t SA_CTL_StreamHandle_t;

typedef uint8_t SA_CTL_RequestID_t;
typedef uint32_t SA_CTL_PropertyKey_t;
typedef uint32_t SA_CTL_Result_t;

typedef struct
{
    uint32_t idx;
    uint32_t type;
    union
    {
        int32_t i32;
        int64_t i64;
        uint8_t unused[24];
    };
} SA_CTL_Event_t;

/**************************
 * MISCELLANEOUS FUNCTIONS *
 **************************/

/**
 @brief Returns the version of the library as a human readable string
 @return The version of the library as a human readable string
 */
SA_CTL_API
const char *SA_CTL_CC SA_CTL_GetFullVersionString();

/**
 @brief Returns a human readable string for the given result code
 @param result Resultcode to get the description for
 @return Human readable string for the given result code
 */
SA_CTL_API
const char *SA_CTL_CC SA_CTL_GetResultInfo(SA_CTL_Result_t result);

/**
 @brief Returns a human readable info string for the given event
 @param event Event to get the description for
 @return Human readable string for the given event
 */
SA_CTL_API
const char *SA_CTL_CC SA_CTL_GetEventInfo(const SA_CTL_Event_t *event);

/***************************
 * INITIALIZATION FUNCTIONS *
 ***************************/

/**
 @brief Opens a connection to a device specified by a locator string
 @param[out] dHandle Handle to the device. Must be passed to following function calls.
 @param locator Specifies the device
 @param config Configuration options for the initialization

 @default config ""

*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_Open(SA_CTL_DeviceHandle_t *dHandle, const char *locator, const char *config);

/**
 @brief Closes a previously established connection to a device
 @param dHandle Handle to the device
 */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Close(SA_CTL_DeviceHandle_t dHandle);

/**
 @brief Aborts all blocking functions
 @param dHandle Handle to the device

 Aborts waiting functions like SA_CTL_WaitForEvent
 - If no thread is currently waiting, the next call to SA_CTL_WaitForEvent
   will be canceled.
 - The unblocked function will return with an SA_CTL_ERROR_CANCELED error.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Cancel(SA_CTL_DeviceHandle_t dHandle);

/**
 @brief Returns a list of locator strings of available devices
 @param options Options for the find procedure
 @param[out] deviceList Buffer for device locators
 @param[in,out] deviceListLen Length of the buffer to allocate

 @default options ""
 @size deviceList ~deviceListLen
 @default deviceListLen 1024
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_FindDevices(const char *options, char *deviceList, size_t *deviceListLen);

/***********************************
 * BLOCKING READ/WRITE ACCESS - i32 *
 ***********************************/

/**
 @brief Directly returns the value of a 32-bit integer (array) property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param[out] value Buffer for the read values
 @param[out,in] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @nullable value alloc
 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in number of elements, not number of bytes) when the function is
   called. On function return it contains the number of values written to the
   buffer. A null pointer is allowed which implicitly indicates an array size of 1.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_i32(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t *value,
    size_t *ioArraySize
);

/**
 @brief Directly sets the value of a 32-bit integer property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_i32(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t value
);

/**
 @brief Directly sets the value of a 32-bit integer array property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param values Buffer containing the values to be written
 @param arraySize Size of the buffer (in number of elements, not number of bytes)

 @size values ~arraySize
 @nullable values alloc
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetPropertyArray_i32(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int32_t *values,
    size_t arraySize
);

/***********************************
 * BLOCKING READ/WRITE ACCESS - i64 *
 ***********************************/

/**
 @brief Directly returns the value of a 64-bit integer (array) property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param[out] value Buffer for the read values
 @param[out,in] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @nullable value alloc

 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in number of elements, not number of bytes) when the function is
   called. On function return it contains the number of values written to the
   buffer. A null pointer is allowed which implicitly indicates an array size of 1.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_i64(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t *value,
    size_t *ioArraySize
);

/**
 @brief Directly sets the value of a 64-bit integer property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_i64(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t value
);

/**
 @brief Directly sets the value of a 64-bit integer array property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param values Buffer containing the values to be written
 @param arraySize Size of the buffer (in number of elements, not number of bytes)

 @size values ~arraySize
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetPropertyArray_i64(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int64_t *values,
    size_t arraySize
);

/**************************************
 * BLOCKING READ/WRITE ACCESS - String *
 **************************************/

/**
 @brief Directly returns the value of a string (array) property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param[out] value Buffer for the read values
 @param[out,in] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @default ioArraySize 64

 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in bytes) when the function is called. On function return it
   contains the number of characters written to the buffer.
 - The null termination of a string implicitly serves as a separator in case
   multiple strings are returned.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_GetProperty_s(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, char *value,
    size_t *ioArraySize
);

/**
 @brief Directly sets the value of a string property
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_SetProperty_s(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const char *value
);

/****************************/
/* NON-BLOCKING READ ACCESS */
/****************************/

/**
 @brief Requests the value of a property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0

 - Received values can be accessed later via the obtained request ID and
   the corresponding SA_CTL_ReadProperty_x functions.
 */
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestReadProperty(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, SA_CTL_RequestID_t *rID,
    SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Reads a 32-bit integer property value (array) that has previously been requested using
 RequestReadProperty
 @param dHandle Handle to the device
 @param rID Request ID
 @param[out] value Buffer for the read values
 @param[in,out] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @nullable value alloc

 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in number of elements, not number of bytes) when the function is
   called. On function return it contains the number of values written to the
   buffer. A null pointer is allowed which implicitly indicates an array size of 1.
 - While the request-function is non-blocking the read-functions block until the
   desired data has arrived.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_i32(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, int32_t *value, size_t *ioArraySize
);

/**
 @brief Reads a 64-bit integer property value (array) that has previously been requested using
 RequestReadProperty
 @param dHandle Handle to the device
 @param rID Request ID
 @param[out] value Buffer for the read values
 @param[in,out] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @nullable value alloc

 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in number of elements, not number of bytes) when the function is
   called. On function return it contains the number of values written to the
   buffer. A null pointer is allowed which implicitly indicates an array size of 1.
 - While the request-function is non-blocking the read-functions block until the
   desired data has arrived.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_i64(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, int64_t *value, size_t *ioArraySize
);

/**
 @brief Reads a string property value (array) that has previously been requested using
 RequestReadProperty
 @param dHandle Handle to the device
 @param rID Request ID
 @param[out] value Buffer for the read values
 @param[in,out] ioArraySize Size of the buffer for values

 @size value ~ioArraySize
 @default ioArraySize 64

 - ioArraySize is a pointer to a size value that must contain the size of the
   value buffer (in bytes) when the function is called. On function return it
   contains the number of characters written to the buffer.
 - The null termination of a string implicitly serves as a separator in case
   multiple strings are returned.
 - While the request-function is non-blocking the read-functions block until
   the desired data has arrived.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_ReadProperty_s(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID, char *value, size_t *ioArraySize
);

/*****************************/
/* NON-BLOCKING WRITE ACCESS */
/*****************************/

/**
 @brief Requests to write the value of a 32-bit integer property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0
 @nullable rID alloc
 - The result (whether the write was successful or not) can be accessed later by
   passing the obtained request ID to the SA_CTL_WaitForWrite function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_i32(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int32_t value,
    SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Requests to write the value of a 64-bit integer property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0
 @nullable rID alloc
 - The result (whether the write was successful or not) can be accessed later by
   passing the obtained request ID to the SA_CTL_WaitForWrite function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_i64(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, int64_t value,
    SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Requests to write the value of a string property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0
 @nullable rID alloc
 - The result (whether the write was successful or not) can be accessed later by
   passing the obtained request ID to the SA_CTL_WaitForWrite function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWriteProperty_s(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const char *value,
    SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Requests to write the value of a 32-bit integer array property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0
 @nullable rID alloc
 - The result (whether the write was successful or not) can be accessed later by
   passing the obtained request ID to the SA_CTL_WaitForWrite function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWritePropertyArray_i32(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int32_t *values,
    size_t arraySize, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Requests to write the value of a 64-bit integer array property (non-blocking)
 @param dHandle Handle to the device
 @param idx Index of the addressed device, module or channel
 @param pkey Key that identifies the property
 @param value Value that should be written
 @param[out] rID Request ID
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0
 @nullable rID alloc
 - The result (whether the write was successful or not) can be accessed later by
   passing the obtained request ID to the SA_CTL_WaitForWrite function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_RequestWritePropertyArray_i64(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_PropertyKey_t pkey, const int64_t *values,
    size_t arraySize, SA_CTL_RequestID_t *rID, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Returns the result of a property write access that has previously been requested using the
 data type specific RequestWriteProperty_x function
 @param dHandle Handle to the device
 @param rID Request ID
 @return Result of a property write access

 - While the request-function is non-blocking the SA_CTL_WaitForWrite function
   blocks until the desired result has arrived.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_WaitForWrite(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID);

/**
 @brief Cancels a non-blocking read or write request
 @param dHandle Handle to the device
 @param rID Request ID

 - Note that without output buffering the request has already been sent. In this
   case only the answer/result will be discarded but property writes will still be
   executed.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CancelRequest(SA_CTL_DeviceHandle_t dHandle, SA_CTL_RequestID_t rID);

/**************************/
/* OUTPUT BUFFER HANDLING */
/**************************/

/**
 @brief Opens an output buffer for delayed transmission of several commands
 @param dHandle Handle to the device
 @param[out] tHandle Handle to a transmit buffer
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CreateOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t *tHandle);

/** Flushes an output buffer and triggers the transmission to the device
 @param dHandle Handle to the device
 @param tHandle Handle to a transmit buffer
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_FlushOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/** Cancels an output buffer and discards all buffered commands
 @param dHandle Handle to the device
 @param tHandle Handle to a transmit buffer
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CancelOutputBuffer(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/**************************/
/* COMMAND GROUP HANDLING */
/**************************/

/**
 @brief Opens a command group that can be used to combine multiple asynchronous commands into an
 atomic group
 @param dHandle Handle to the device
 @param[out] tHandle Handle to a transmit buffer
 @param triggerMode Trigger mode for this command group

 - The trigger mode for a command group must be either
   SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT (0) or SA_CTL_CMD_GROUP_TRIGGER_MODE_EXTERNAL (1).
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_OpenCommandGroup(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t *tHandle, uint32_t triggerMode
);

/**
 @brief Closes and eventually executes the assembled command group depending on the configured
 trigger mode
 @param dHandle Handle to the device
 @param tHandle Handle to a transmit buffer
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CloseCommandGroup(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/**
 @brief Discards all buffered commands and releases the associated transmit handle
 @param dHandle Handle to the device
 @param tHandle Handle to a transmit buffer
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CancelCommandGroup(SA_CTL_DeviceHandle_t dHandle, SA_CTL_TransmitHandle_t tHandle);

/***************************/
/* WAIT AND RECEIVE EVENTS */
/***************************/

/**
 @brief Listens to events from the device
 @param dHandle Handle to the device
 @param[out] event Event that occurred
 @param timeout Maximum time to wait for an event to occur

 - The timeout is given in milliseconds. The special value SA_CTL_INFINITE is also valid.
   Setting the timeout to zero will check for already queued events, but does not
   block if no event is available.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_WaitForEvent(SA_CTL_DeviceHandle_t dHandle, SA_CTL_Event_t *event, uint32_t timeout);

/**********************/
/* MOVEMENT FUNCTIONS */
/**********************/

/**
 @brief Starts a calibration routine for a given channel
 @param dHandle Handle to the device
 @param idx Index of the channel
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0

 - The calibration options property should be configured to define
   the behavior of the calibration sequence before calling this function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_Calibrate(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/**
 @brief Starts a referencing routine for a given channel
 @param dHandle Handle to the device
 @param idx Index of the channel
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0

 - The referencing options property (as well as the move velocity, acceleration, etc.)
   should be configured to define the behavior of the referencing sequence
   before calling this function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_Reference(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/**
 @brief Instructs a positioner to move according to the current move configuration
 @param dHandle Handle to the device
 @param idx Index of the channel
 @param moveValue The interpretation depends on the configured move mode
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0

 - The move mode as well as corresponding parameters (e.g. frequency, move velocity,
   holdTime, etc.) have to be configured before calling this function.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_Move(
    SA_CTL_DeviceHandle_t dHandle, int8_t idx, int64_t moveValue, SA_CTL_TransmitHandle_t tHandle
);

/**
 @brief Stops any ongoing movement of the given channel
 @param dHandle Handle to the device
 @param idx Index of the channel
 @param tHandle Optional handle to a transmit buffer. If unused set to zero.

 @default tHandle 0

 - Note that the function call returns immediately, without waiting for the stop to complete.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_Stop(SA_CTL_DeviceHandle_t dHandle, int8_t idx, SA_CTL_TransmitHandle_t tHandle);

/***********************/
/* STREAMING FUNCTIONS */
/***********************/

/**
 @brief Opens a trajectory stream to the device
 @param dHandle Handle to the device
 @param[out] sHandle Stream Handle
 @param triggerMode Trigger mode for the trajectory stream

 - The trigger mode for the trajectory stream must be one of
     - SA_CTL_STREAM_TRIGGER_MODE_DIRECT (0),
     - SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_ONCE (1),
     - SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_SYNC (2),
     - SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL (3).
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_OpenStream(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t *sHandle, uint32_t triggerMode
);

/**
 @brief Supplies the device with stream data by sending one frame per function call
 @param dHandle Handle to the device
 @param sHandle Stream Handle
 @param frameData Frame data buffer
 @param frameSize Size of the frame data

 @size frameData ~frameSize

 - A frame contains the data for one interpolation point which must be assembled
   by concatenating elements of channel index (1 byte) and position (8 byte).
 - This function may block if the flow control needs to throttle the data rate.
   The function returns as soon as the frame was transmitted to the controller.
 - The desired streamrate as well as the external syncrate have to be configured beforehand.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC SA_CTL_StreamFrame(
    SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle, const uint8_t *frameData,
    uint32_t frameSize
);

/**
 @brief Closes a trajectory stream
 @param dHandle Handle to the device
 @param sHandle Stream Handle

 - If the stream is not closed properly, the device will generate a buffer underflow
   error after the last frame has been processed.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_CloseStream(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle);

/**
 @brief Aborts a trajectory stream
 @param dHandle Handle to the device
 @param sHandle Stream Handle

 - All movements are stopped immediately and remaining buffered interpolation points are discarded.
*/
SA_CTL_API
SA_CTL_Result_t SA_CTL_CC
SA_CTL_AbortStream(SA_CTL_DeviceHandle_t dHandle, SA_CTL_StreamHandle_t sHandle);

#ifdef __cplusplus
}
#endif

#endif // SMARACT_CTL_H
